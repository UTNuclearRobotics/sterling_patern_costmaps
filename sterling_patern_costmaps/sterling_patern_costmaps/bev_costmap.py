import os

import numpy as np
import torch

from sterling_patern_costmaps.train_patern import PaternPreAdaptation


class BEVCostmap:
    """
    An overview of the cost inference process for local planning at deployment using trained preference predictor.
    """

    def __init__(self, model_path, adapted=False, label_obstacles=False):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.label_obstacles = label_obstacles
        
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
        if isinstance(cells, np.ndarray):
            cells = torch.tensor(cells, dtype=torch.float32, device=self.device)

        if len(cells.shape) == 4:  # [B, C, H, W]
            pass
        elif len(cells.shape) == 3:  # [C, H, W] -> [1, C, H, W]
            cells = cells.unsqueeze(0)

        with torch.no_grad():
            # Pass None for inertial data
            phi_vis, _, uvis_pred, _, final_cost = self.model(cells, inertial=None)

            # preferences = preferences * 255
            uvis_costs = uvis_pred.squeeze(-1).cpu().numpy().astype(np.uint8)
            final_costs = final_cost.squeeze(-1).cpu().numpy().astype(np.uint8)
            
            # Make sure the costs never return an obstacle (value of 100)
            if not self.label_obstacles:
                final_costs = np.clip(final_costs, -1, 99)
            
            return uvis_costs, final_costs

    def BEV_to_costmap(self, bev_img, cell_size):
        """Convert BEV image to costmap while automatically marking consistent black areas."""
        height, width = bev_img.shape[:2]
        num_cells_y, num_cells_x = height // cell_size, width // cell_size

        # Determine effective dimensions that are multiples of cell_size.
        effective_height = num_cells_y * cell_size
        effective_width = num_cells_x * cell_size

        # Slice the image to the effective region.
        bev_img = bev_img[:effective_height, :effective_width]

        # Initialize costmap container.
        costmap = np.empty((num_cells_y, num_cells_x), dtype=np.int8)

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
        
        # Rearrange to (num_cells_y, num_cells_x, channels, cell_size, cell_size)
        cells = cells.transpose(0, 1, 4, 2, 3)

        # Select only valid (non-black) cells.
        valid_cells = cells[~black_cells]

        # Calculate costs for valid cells in a single batch.
        if valid_cells.size:
            # Ensure valid_cells has shape [B, C, H, W]
            if len(valid_cells.shape) == 5 and valid_cells.shape[2] == 1:  # Grayscale
                valid_cells = valid_cells.squeeze(2)  # [B, H, W]
                valid_cells = np.stack([valid_cells] * 3, axis=1)  # [B, 3, H, W]
            uvis_cost, final_cost = self.predict_preferences(valid_cells)
        else:
            uvis_cost, final_cost = np.empty((0,), dtype=np.uint8)

        # Assemble costmap: assign maximum cost (-1) to unknown cells and computed costs to others.
        costmap[black_cells] = -1
        costmap[~black_cells] = final_cost
        
        print(costmap)

        return costmap
