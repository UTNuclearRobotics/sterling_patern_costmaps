import os

import numpy as np
import torch
import time
from sterling_patern_costmaps.train_patern import PaternPreAdaptation


class BEVCostmap:
    """
    An overview of the cost inference process for local planning at deployment using trained preference predictor.
    """

    def __init__(self, model_path, adapted=False, label_obstacles=False, logger=None):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.label_obstacles = label_obstacles
        self.logger = logger
        
        # Load visual encoder model weights
        self.model = PaternPreAdaptation(self.device).to(self.device)

        # Define the expected .pt files for each submodule
        if adapted:
            weight_files = {
                "visual_encoder": "fvis_adapted.pt",
                "proprioceptive_encoder": "fpro.pt",
                "uvis": "uvis_adapted.pt",
                "upro": "upro.pt",
                "cost_head": "cost_head_adapted.pt",
            }
        else:
            weight_files = {
                "visual_encoder": "fvis.pt",
                "proprioceptive_encoder": "fpro.pt",
                "uvis": "uvis.pt",
                "upro": "upro.pt",
                "cost_head": "cost_head.pt",
            }

        # Load weights for each submodule
        for submodule_name, file_name in weight_files.items():
            file_path = os.path.join(model_path, file_name)
            if not os.path.exists(file_path):
                raise FileNotFoundError(f"Weight file for {submodule_name} not found at: {file_path}")

            # Load the state dict for the submodule
            state_dict = torch.load(file_path, weights_only=True, map_location=self.device)

            # Get the corresponding submodule from self.model
            submodule = getattr(self.model, submodule_name)
            submodule.load_state_dict(state_dict)
            print(f"Loaded {submodule_name} weights from {file_path}")

        # Set the model to evaluation mode
        self.model.eval()

    def predict_preferences(self, cells):
        """Predict preferences for a batch of cells using the trained uvis model."""
        import time
        
        t_start = time.time()
        
        if isinstance(cells, np.ndarray):
            cells = torch.tensor(cells, dtype=torch.float32, device=self.device)
        
        t_transfer_to_gpu = time.time()
        if self.logger:
            self.logger.info(f"    CPU→GPU transfer: {(t_transfer_to_gpu - t_start)*1000:.2f}ms")

        if len(cells.shape) == 4:
            pass
        elif len(cells.shape) == 3:
            cells = cells.unsqueeze(0)

        with torch.no_grad():
            phi_vis, _, uvis_pred, _, final_cost = self.model(cells, inertial=None)
            
            t_inference = time.time()
            if self.logger:
                self.logger.info(f"    GPU inference: {(t_inference - t_transfer_to_gpu)*1000:.2f}ms")

            uvis_costs = uvis_pred.squeeze(-1).cpu().numpy().astype(np.uint8)
            final_costs = final_cost.squeeze(-1).cpu().numpy().astype(np.uint8)
            
            t_transfer_to_cpu = time.time()
            if self.logger:
                self.logger.info(f"    GPU→CPU transfer: {(t_transfer_to_cpu - t_inference)*1000:.2f}ms")
            
            if not self.label_obstacles:
                final_costs = np.clip(final_costs, -1, 99)
                
        return uvis_costs, final_costs

    def BEV_to_costmap(self, bev_img, cell_size):
        """Convert BEV image to costmap while automatically marking consistent black areas."""
        t_start = time.time()
        height, width = bev_img.shape[:2]
        num_cells_y, num_cells_x = height // cell_size, width // cell_size

        # Determine effective dimensions that are multiples of cell_size.
        effective_height = num_cells_y * cell_size
        effective_width = num_cells_x * cell_size

        # Slice the image to the effective region (this is a view, no copy).
        bev_img = bev_img[:effective_height, :effective_width]

        # Handle grayscale to RGB conversion early if needed (assumes model expects [B, 3, H, W]).
        if bev_img.ndim == 2:  # Grayscale (H, W) -> (H, W, 1)
            bev_img = bev_img[..., np.newaxis]
        if bev_img.shape[2] == 1:  # (H, W, 1) -> (H, W, 3)
            bev_img = np.repeat(bev_img, 3, axis=2)

        t_preprocessing = time.time()
        if self.logger:
            self.logger.info(f"  Preprocessing: {(t_preprocessing - t_start)*1000:.2f}ms")

        # Create mask for black regions.
        black_cells = np.zeros((num_cells_y, num_cells_x), dtype=bool)
        black_cells[-2, [0, -1]] = True  # Row -2, columns 0 and -1
        black_cells[-1, [0, 1, -2, -1]] = True  # Row -1, columns 0, 1, -2, and -1

        # Use stride tricks to extract cell views without copying data.
        channels = bev_img.shape[2]
        cell_shape = (num_cells_y, num_cells_x, cell_size, cell_size, channels)
        cell_strides = (
            bev_img.strides[0] * cell_size,
            bev_img.strides[1] * cell_size,
            bev_img.strides[0],
            bev_img.strides[1],
            bev_img.strides[2],
        )
        cells = np.lib.stride_tricks.as_strided(bev_img, shape=cell_shape, strides=cell_strides)
        
        # Rearrange to (num_cells_y, num_cells_x, channels, cell_size, cell_size) – this is a view.
        cells = cells.transpose(0, 1, 4, 2, 3)

        # Flatten to batch shape for prediction: (num_cells_y * num_cells_x, channels, cell_size, cell_size) – still a view.
        all_cells = cells.reshape(-1, channels, cell_size, cell_size)

        t_extraction = time.time()
        if self.logger:
            self.logger.info(f"  Patch extraction: {(t_extraction - t_preprocessing)*1000:.2f}ms")

        # Calculate costs for all cells in a single batch.
        if all_cells.size:
            uvis_cost, final_cost = self.predict_preferences(all_cells)
        else:
            final_cost = np.empty((0,), dtype=np.uint8)

        t_inference = time.time()
        if self.logger:
            self.logger.info(f"  Model inference: {(t_inference - t_extraction)*1000:.2f}ms")

        # Assemble costmap: reshape final_cost and override black cells with -1.
        costmap = final_cost.reshape(num_cells_y, num_cells_x).astype(np.int8)
        costmap[black_cells] = -1

        t_assembly = time.time()
        if self.logger:
            self.logger.info(f"  Costmap assembly: {(t_assembly - t_inference)*1000:.2f}ms")
            self.logger.info(f"  TOTAL BEV_to_costmap: {(t_assembly - t_start)*1000:.2f}ms")

        return costmap
