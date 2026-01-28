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
