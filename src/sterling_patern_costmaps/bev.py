import cv2
import numpy as np


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
    # print(f"Stitched image size: {stitched_image.shape}")

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
    Ultra-fast GPU BEV generation using pre-computed remap and a SINGLE GPU operation.
    Fully vectorized - no loops!
    """
    try:
        if logger:
            logger.info("=" * 60)
            logger.info("DEBUG: Starting get_BEV_image_gpu")
            logger.info(f"DEBUG: Input image shape: {image.shape}, dtype: {image.dtype}")
            logger.info(f"DEBUG: H shape: {H.shape}, dtype: {H.dtype}")
        
        if cv2.cuda.getCudaEnabledDeviceCount() == 0:
            raise RuntimeError("No CUDA device found - cannot use GPU version")
        if logger:
            logger.info("DEBUG: CUDA device found")

        rows, cols = grid_size
        patch_width, patch_height = patch_size
        origin_shift = (patch_size[0], patch_size[1] * 2 + 60)

        total_width = cols * patch_width
        total_height = rows * patch_height
        if logger:
            logger.info(f"DEBUG: Output dimensions: {total_width}x{total_height}")

        # Vectorized homography computation
        if logger:
            logger.info("DEBUG: Computing homographies...")
        i_range = np.arange(-rows // 2, rows // 2)
        j_range = np.arange(-cols // 2, cols // 2)
        j_grid, i_grid = np.meshgrid(j_range, i_range)
        
        x_shifts = j_grid * patch_size[0] + origin_shift[0]
        y_shifts = i_grid * patch_size[1] + origin_shift[1]
        
        T_shifts = np.zeros((rows, cols, 3, 3), dtype=np.float32)
        T_shifts[:, :, 0, 0] = 1
        T_shifts[:, :, 1, 1] = 1
        T_shifts[:, :, 2, 2] = 1
        T_shifts[:, :, 0, 2] = x_shifts
        T_shifts[:, :, 1, 2] = y_shifts
        
        if logger:
            logger.info(f"DEBUG: T_shifts shape: {T_shifts.shape}")
            logger.info("DEBUG: Computing H_shifted_all...")
        H_shifted_all = T_shifts @ H[np.newaxis, np.newaxis, :, :]
        
        if logger:
            logger.info(f"DEBUG: H_shifted_all shape: {H_shifted_all.shape}")
            logger.info("DEBUG: Computing inverse homographies...")
        H_inv_all = np.linalg.inv(H_shifted_all)
        
        if logger:
            logger.info(f"DEBUG: H_inv_all shape: {H_inv_all.shape}")
            logger.info("DEBUG: Creating coordinate grids...")
        y_coords, x_coords = np.mgrid[0:total_height, 0:total_width].astype(np.float32)
        
        if logger:
            logger.info("DEBUG: Computing patch indices...")
        patch_row_idx = y_coords // patch_height
        patch_col_idx = x_coords // patch_width
        actual_row_idx = rows - 1 - patch_row_idx
        actual_col_idx = cols - 1 - patch_col_idx
        
        local_x = x_coords % patch_width
        local_y = y_coords % patch_height
        
        if logger:
            logger.info("DEBUG: Indexing H_inv per pixel...")
        H_inv_per_pixel = H_inv_all[actual_row_idx.astype(int), actual_col_idx.astype(int)]
        
        if logger:
            logger.info(f"DEBUG: H_inv_per_pixel shape: {H_inv_per_pixel.shape}")
            logger.info("DEBUG: Creating homogeneous coordinates...")
        local_coords = np.stack([local_x, local_y, np.ones_like(local_x)], axis=-1)[..., np.newaxis]
        
        if logger:
            logger.info("DEBUG: Applying inverse homography...")
        transformed = H_inv_per_pixel @ local_coords
        transformed = transformed.squeeze(-1)
        
        if logger:
            logger.info("DEBUG: Converting to Cartesian coordinates...")
        map_x = (transformed[:, :, 0] / transformed[:, :, 2]).astype(np.float32)
        map_y = (transformed[:, :, 1] / transformed[:, :, 2]).astype(np.float32)
        map_x = np.ascontiguousarray(map_x)
        map_y = np.ascontiguousarray(map_y)

        if logger:
            logger.info(f"DEBUG: map_x min/max: {map_x.min()}/{map_x.max()}")
            logger.info(f"DEBUG: map_y min/max: {map_y.min()}/{map_y.max()}")
            logger.info(f"DEBUG: map_x has NaN: {np.isnan(map_x).any()}")
            logger.info(f"DEBUG: map_y has NaN: {np.isnan(map_y).any()}")
            logger.info(f"DEBUG: map_x has Inf: {np.isinf(map_x).any()}")
            logger.info(f"DEBUG: map_y has Inf: {np.isinf(map_y).any()}")
        
        if logger:
            logger.info(f"DEBUG: Maps ready - shape: {map_x.shape}")
            logger.info("DEBUG: Uploading to GPU...")
        
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(image)
        
        gpu_map_x = cv2.cuda_GpuMat()
        gpu_map_x.upload(map_x)
        
        gpu_map_y = cv2.cuda_GpuMat()
        gpu_map_y.upload(map_y)
        
        if logger:
            logger.info("DEBUG: Performing GPU remap...")
        gpu_output = cv2.cuda_GpuMat(total_height, total_width, gpu_img.type())

        # Try without specifying borderValue first
        cv2.cuda.remap(
            gpu_img,
            gpu_map_x,
            gpu_map_y,
            cv2.INTER_LINEAR,
            gpu_output,
            cv2.BORDER_CONSTANT
        )

        if logger:
            logger.info(f"DEBUG: GPU remap complete, output empty: {gpu_output.empty()}")
            logger.info(f"DEBUG: GPU output size: {gpu_output.size()}")
            logger.info(f"DEBUG: GPU output channels: {gpu_output.channels()}")
            logger.info("DEBUG: Downloading result...")

        stitched_image = gpu_output.download()

        if logger:
            logger.info(f"DEBUG: Download returned None: {stitched_image is None}")
            if stitched_image is not None:
                logger.info(f"DEBUG: Success! Shape: {stitched_image.shape}")
            else:
                logger.error("DEBUG: Download returned None - checking if output is empty")
        
        if logger:
            logger.info(f"DEBUG: Success! Shape: {stitched_image.shape}")
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

    Args:
        image (numpy.ndarray): The input image (BGR format).
        H (numpy.ndarray): Homography matrix used to extract the patch.
        patch_size (tuple): Size of the patch (width, height).
        color (tuple): Color of the rectangle in BGR format (default: green).
        thickness (int): Thickness of the rectangle border (default: 2 pixels).

    Returns:
        numpy.ndarray: The image with patch boundaries drawn.
    """
    # Make a copy of the image to avoid modifying the original
    output_image = image.copy()

    # Define the corners of the patch in the patch coordinate system (homogeneous coordinates)
    w, h = patch_size
    patch_corners = np.array([
        [0, 0, 1],  # Top-left
        [w, 0, 1],  # Top-right
        [w, h, 1],  # Bottom-right
        [0, h, 1]   # Bottom-left
    ], dtype=np.float32).T  # Shape: (3, 4)

    # Compute the inverse homography to map patch corners back to the original image
    H_inv = np.linalg.inv(H)
    image_corners = H_inv @ patch_corners  # Shape: (3, 4)
    
    # Normalize homogeneous coordinates (x, y, w) -> (x/w, y/w)
    image_corners = image_corners[:2] / image_corners[2:3]  # Shape: (2, 4)
    image_corners = image_corners.T  # Shape: (4, 2), each row is (x, y)

    # Convert to integer coordinates for drawing
    image_corners = image_corners.astype(int)

    # Draw the rectangle by connecting the corners
    for i in range(4):
        pt1 = tuple(image_corners[i])
        pt2 = tuple(image_corners[(i + 1) % 4])  # Connect to the next corner, wrap around at 4
        cv2.line(output_image, pt1, pt2, color, thickness)

    return output_image
