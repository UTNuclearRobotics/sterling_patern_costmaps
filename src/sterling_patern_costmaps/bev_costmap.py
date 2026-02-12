import os

import numpy as np
import torch
import time
from sterling_patern_costmaps.train_patern import PaternPreAdaptation

class BEVCostmapGPU:
    """
    GPU-accelerated cost inference pipeline.
    Keeps all data on GPU from BEV image to final costmap.
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

            state_dict = torch.load(file_path, weights_only=True, map_location=self.device)
            submodule = getattr(self.model, submodule_name)
            submodule.load_state_dict(state_dict)
            if self.logger:
                self.logger.info(f"Loaded {submodule_name} weights from {file_path}")

        # Set the model to evaluation mode
        self.model.eval()

    def predict_preferences(self, cells_tensor):
        """
        Predict preferences for a batch of cells.
        
        Args:
            cells_tensor: torch.Tensor already on GPU, shape [B, C, H, W]
        
        Returns:
            uvis_costs, final_costs as torch tensors on GPU
        """
        with torch.no_grad():
            phi_vis, _, uvis_pred, _, final_cost = self.model(cells_tensor, inertial=None)
            
            uvis_costs = uvis_pred.squeeze(-1)
            final_costs = final_cost.squeeze(-1)
            
            # Clipping happens on GPU
            if not self.label_obstacles:
                final_costs = torch.clamp(final_costs, -1, 99)
            
            return uvis_costs, final_costs

    def extract_patches_gpu(self, gpu_bev_img, cell_size):
        """
        Extract patches from GPU BEV image using GPU operations.
        
        Args:
            gpu_bev_img: cv2.cuda_GpuMat containing BEV image
            cell_size: Size of each patch
        
        Returns:
            torch.Tensor on GPU with shape [num_patches, 3, cell_size, cell_size]
        """
        import time
        t_start = time.time()
        
        # Download BEV to CPU for now (we'll optimize this later)
        bev_cpu = gpu_bev_img.download()
        
        if self.logger:
            t_download = time.time()
            self.logger.info(f"    BEV download: {(t_download - t_start)*1000:.2f}ms")
        
        height, width = bev_cpu.shape[:2]
        num_cells_y, num_cells_x = height // cell_size, width // cell_size

        effective_height = num_cells_y * cell_size
        effective_width = num_cells_x * cell_size
        bev_cpu = bev_cpu[:effective_height, :effective_width]

        # Handle grayscale to RGB conversion
        if bev_cpu.ndim == 2:
            bev_cpu = bev_cpu[..., np.newaxis]
        if bev_cpu.shape[2] == 1:
            bev_cpu = np.repeat(bev_cpu, 3, axis=2)

        # Use stride tricks to extract patches (still on CPU for now)
        channels = bev_cpu.shape[2]
        cell_shape = (num_cells_y, num_cells_x, cell_size, cell_size, channels)
        cell_strides = (
            bev_cpu.strides[0] * cell_size,
            bev_cpu.strides[1] * cell_size,
            bev_cpu.strides[0],
            bev_cpu.strides[1],
            bev_cpu.strides[2],
        )
        cells = np.lib.stride_tricks.as_strided(bev_cpu, shape=cell_shape, strides=cell_strides)
        cells = cells.transpose(0, 1, 4, 2, 3)  # (rows, cols, C, H, W)
        all_cells = cells.reshape(-1, channels, cell_size, cell_size)  # (N, C, H, W)
        
        if self.logger:
            t_extract = time.time()
            self.logger.info(f"    Patch extraction (CPU): {(t_extract - t_download)*1000:.2f}ms")
        
        # Convert to torch tensor and move to GPU in one operation
        cells_tensor = torch.from_numpy(all_cells).float().to(self.device, non_blocking=True)
        
        if self.logger:
            t_upload = time.time()
            self.logger.info(f"    Upload to GPU: {(t_upload - t_extract)*1000:.2f}ms")
        
        return cells_tensor, num_cells_y, num_cells_x

    def BEV_to_costmap_gpu(self, gpu_bev_img, cell_size):
        """
        Convert GPU BEV image to costmap, keeping everything on GPU.
        
        Args:
            gpu_bev_img: cv2.cuda_GpuMat containing BEV image
            cell_size: Size of each cell in pixels
        
        Returns:
            costmap as numpy array (int8)
        """
        import time
        t_start = time.time()
        
        # Extract patches and get as torch tensor on GPU
        cells_tensor, num_cells_y, num_cells_x = self.extract_patches_gpu(gpu_bev_img, cell_size)
        
        if self.logger:
            t_patches = time.time()
            self.logger.info(f"  GPU patch extraction total: {(t_patches - t_start)*1000:.2f}ms")
        
        # Run inference (already on GPU)
        if cells_tensor.size(0) > 0:
            uvis_cost, final_cost = self.predict_preferences(cells_tensor)
        else:
            final_cost = torch.empty((0,), dtype=torch.uint8, device=self.device)
        
        if self.logger:
            t_inference = time.time()
            self.logger.info(f"  GPU inference: {(t_inference - t_patches)*1000:.2f}ms")
        
        # Move to CPU only at the end
        final_cost_cpu = final_cost.cpu().numpy().astype(np.uint8)
        
        if self.logger:
            t_download = time.time()
            self.logger.info(f"  Download costmap: {(t_download - t_inference)*1000:.2f}ms")
        
        # Assemble costmap
        costmap = final_cost_cpu.reshape(num_cells_y, num_cells_x).astype(np.int8)
        
        # Mark black cells
        black_cells = np.zeros((num_cells_y, num_cells_x), dtype=bool)
        black_cells[-2, [0, -1]] = True
        black_cells[-1, [0, 1, -2, -1]] = True
        costmap[black_cells] = -1
        
        if self.logger:
            t_end = time.time()
            self.logger.info(f"  Costmap assembly: {(t_end - t_download)*1000:.2f}ms")
            self.logger.info(f"  TOTAL BEV_to_costmap_gpu: {(t_end - t_start)*1000:.2f}ms")
        
        return costmap

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
