import cv2
import numpy as np


class BEVTransformer:
    """
    Caches remap maps for GPU BEV generation to avoid recomputing them every frame.
    The maps only depend on H, patch_size, and grid_size which are constant.
    """
    def __init__(self):
        self.cached_maps = {}
    
    def _compute_cache_key(self, H, patch_size, grid_size):
        """Create a hashable cache key from parameters"""
        H_bytes = H.tobytes()
        return (H_bytes, patch_size, grid_size)
    
    def get_or_create_maps(self, H, patch_size, grid_size, logger=None):
        """
        Get cached GPU remap maps or compute them if not cached.
        Returns: (gpu_map_x, gpu_map_y, total_width, total_height)
        """
        cache_key = self._compute_cache_key(H, patch_size, grid_size)
        
        if cache_key in self.cached_maps:
            if logger:
                logger.info("DEBUG: Using cached remap maps")
            return self.cached_maps[cache_key]
        
        if logger:
            logger.info("DEBUG: Computing remap maps (first time only)...")
        
        rows, cols = grid_size
        patch_width, patch_height = patch_size
        origin_shift = (patch_size[0], patch_size[1] * 2 + 60)
        
        total_width = cols * patch_width
        total_height = rows * patch_height
        
        # Vectorized homography computation
        i_range = np.arange(-rows // 2, rows // 2)
        j_range = np.arange(-cols // 2, cols // 2)
        j_grid, i_grid = np.meshgrid(j_range, i_range)
        
        x_shifts = j_grid * patch_size[0] + origin_shift[0]
        y_shifts = i_grid * patch_size[1] + origin_shift[1]
        
        # Create all translation matrices at once
        T_shifts = np.zeros((rows, cols, 3, 3), dtype=np.float32)
        T_shifts[:, :, 0, 0] = 1
        T_shifts[:, :, 1, 1] = 1
        T_shifts[:, :, 2, 2] = 1
        T_shifts[:, :, 0, 2] = x_shifts
        T_shifts[:, :, 1, 2] = y_shifts
        
        # Batch matrix multiply
        H_shifted_all = T_shifts @ H[np.newaxis, np.newaxis, :, :]
        
        # Compute inverse homographies
        H_inv_all = np.linalg.inv(H_shifted_all)
        
        # Create coordinate grids
        y_coords, x_coords = np.mgrid[0:total_height, 0:total_width].astype(np.float32)
        
        # Determine which patch each pixel belongs to
        patch_row_idx = y_coords // patch_height
        patch_col_idx = x_coords // patch_width
        
        # Account for reversal
        actual_row_idx = rows - 1 - patch_row_idx
        actual_col_idx = cols - 1 - patch_col_idx
        
        # Local coordinates within each patch
        local_x = x_coords % patch_width
        local_y = y_coords % patch_height
        
        # Get the appropriate H_inv for each pixel
        H_inv_per_pixel = H_inv_all[actual_row_idx.astype(int), actual_col_idx.astype(int)]
        
        # Create homogeneous coordinates
        local_coords = np.stack([local_x, local_y, np.ones_like(local_x)], axis=-1)[..., np.newaxis]
        
        # Apply inverse homography
        transformed = H_inv_per_pixel @ local_coords
        transformed = transformed.squeeze(-1)
        
        # Convert from homogeneous to Cartesian
        map_x = (transformed[:, :, 0] / transformed[:, :, 2]).astype(np.float32)
        map_y = (transformed[:, :, 1] / transformed[:, :, 2]).astype(np.float32)
        map_x = np.ascontiguousarray(map_x)
        map_y = np.ascontiguousarray(map_y)
        
        # Upload to GPU and cache
        gpu_map_x = cv2.cuda_GpuMat()
        gpu_map_x.upload(map_x)
        
        gpu_map_y = cv2.cuda_GpuMat()
        gpu_map_y.upload(map_y)
        
        # Cache the GPU maps (they stay on GPU!)
        self.cached_maps[cache_key] = (gpu_map_x, gpu_map_y, total_width, total_height)
        
        if logger:
            logger.info("DEBUG: Remap maps computed and cached on GPU")
        
        return gpu_map_x, gpu_map_y, total_width, total_height


# Global instance for caching
_bev_transformer = BEVTransformer()


def get_BEV_image(image, H, patch_size=(128, 128), grid_size=(7, 12), visualize=False):
    annotated_image = image.copy()
    rows, cols = grid_size
    patch_width, patch_height = patch_size

    # TODO: Don't make this adjust manual
    origin_shift = (patch_size[0], patch_size[1] * 2 + 60)

    row_images = []
    for i in range(-rows // 2, rows // 2):
        col_patches = []
        for j in range(-cols // 2, cols // 2):
            x_shift = j * patch_size[0] + origin_shift[0]
            y_shift = i * patch_size[1] + origin_shift[1]

            # Compute the translated homography
            T_shift = np.array([[1, 0, x_shift], [0, 1, y_shift], [0, 0, 1]])
            H_shifted = T_shift @ H

            # Warp and resize the patch
            cur_patch = cv2.warpPerspective(image, H_shifted, dsize=patch_size)
            if cur_patch.shape != patch_size:
                cur_patch = cv2.resize(cur_patch, patch_size)
            col_patches.append(cur_patch)

            if visualize:
                annotated_image = draw_points(annotated_image, H_shifted, patch_size, color=(0, 255, 0), thickness=2)

        row_image = cv2.hconcat(col_patches[::-1])
        row_images.append(row_image)
    stitched_image = cv2.vconcat(row_images[::-1])

    if visualize:
        cv2.imshow("Current Image with patches", annotated_image)

        # Draw the green grid lines
        for i in range(rows + 1):
            start_point = (0, i * patch_height)
            end_point = (cols * patch_width, i * patch_height)
            stitched_image = cv2.line(stitched_image, start_point, end_point, (0, 255, 0), 2)
        for j in range(cols + 1):
            start_point = (j * patch_width, 0)
            end_point = (j * patch_width, rows * patch_height)
            stitched_image = cv2.line(stitched_image, start_point, end_point, (0, 255, 0), 2)
        cv2.imshow("Stitched BEV Image", stitched_image)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
        exit()

    return stitched_image


def get_BEV_image_gpu(image, H, patch_size=(128, 128), grid_size=(7, 12), visualize=False, logger=None):
    """
    Ultra-fast GPU BEV generation using cached remap maps.
    Maps are computed once and reused for all subsequent frames.
    """
    try:
        if cv2.cuda.getCudaEnabledDeviceCount() == 0:
            raise RuntimeError("No CUDA device found - cannot use GPU version")
        
        # Get or create cached maps (fast on subsequent calls)
        gpu_map_x, gpu_map_y, total_width, total_height = _bev_transformer.get_or_create_maps(
            H, patch_size, grid_size, logger
        )
        
        # Upload image to GPU
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(image)
        
        # Pre-allocate output
        gpu_output = cv2.cuda_GpuMat(total_height, total_width, gpu_img.type())
        
        # Single GPU remap operation
        cv2.cuda.remap(
            gpu_img,
            gpu_map_x,
            gpu_map_y,
            cv2.INTER_LINEAR,
            gpu_output,
            cv2.BORDER_CONSTANT
        )
        
        # Download result
        stitched_image = gpu_output.download()
        
        if visualize:
            annotated_image = image.copy()
            rows, cols = grid_size
            patch_width, patch_height = patch_size
            origin_shift = (patch_size[0], patch_size[1] * 2 + 60)
            
            # Visualize patch boundaries
            for i in range(-rows // 2, rows // 2):
                for j in range(-cols // 2, cols // 2):
                    x_shift = j * patch_width + origin_shift[0]
                    y_shift = i * patch_height + origin_shift[1]
                    T_shift = np.array([[1, 0, x_shift], [0, 1, y_shift], [0, 0, 1]], dtype=np.float32)
                    H_shifted = T_shift @ H
                    annotated_image = draw_points(annotated_image, H_shifted, patch_size,
                                                 color=(0, 255, 0), thickness=2)

            cv2.imshow("Current Image with patches", annotated_image)

            # Draw grid lines
            grid_img = stitched_image.copy()
            for i in range(rows + 1):
                cv2.line(grid_img, (0, i * patch_height), (total_width, i * patch_height), (0, 255, 0), 2)
            for j in range(cols + 1):
                cv2.line(grid_img, (j * patch_width, 0), (j * patch_width, total_height), (0, 255, 0), 2)

            cv2.imshow("Stitched BEV Image", grid_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return stitched_image
        
    except Exception as e:
        if logger:
            logger.error(f"ERROR in get_BEV_image_gpu: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return None


def draw_points(image, H, patch_size=(128, 128), color=(0, 255, 0), thickness=2):
    """
    Draw the boundaries of patches extracted via homography on the original image.
    """
    output_image = image.copy()

    w, h = patch_size
    patch_corners = np.array([
        [0, 0, 1],
        [w, 0, 1],
        [w, h, 1],
        [0, h, 1]
    ], dtype=np.float32).T

    H_inv = np.linalg.inv(H)
    image_corners = H_inv @ patch_corners
    
    image_corners = image_corners[:2] / image_corners[2:3]
    image_corners = image_corners.T
    image_corners = image_corners.astype(int)

    for i in range(4):
        pt1 = tuple(image_corners[i])
        pt2 = tuple(image_corners[(i + 1) % 4])
        cv2.line(output_image, pt1, pt2, color, thickness)

    return output_image
