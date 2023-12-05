import numpy as np

def find_transformation_matrix(src_points, dst_points):
    """
    寻找从源平面到目标平面的变换矩阵
    :param src_points: 源平面上的三个点的坐标，每行一个点
    :param dst_points: 目标平面上对应的三个点的坐标，每行一个点
    :return: 4x4的变换矩阵
    """
    # 构造增广矩阵，每个点的坐标扩展为齐次坐标（增加一维，设为1）
    src_points = np.column_stack((src_points, np.ones(src_points.shape[0])))
    dst_points = np.column_stack((dst_points, np.ones(dst_points.shape[0])))

    # 利用最小二乘法求解变换矩阵
    transformation_matrix, residuals, _, _ = np.linalg.lstsq(src_points, dst_points, rcond=None)

    # 将变换矩阵转为3x3形式
    transformation_matrix = np.vstack([transformation_matrix[:3, :], [0, 0, 0, 1]])

    return transformation_matrix

# 举例：假设有两个平面上的三个点
src_points = np.array([[33,-16.65,0], [33, 16.65, 0], [55, 16.65, 0]])
dst_points = np.array([[44,-20.65,-14], [41,24.65,-13], [56.5,23.65,-23]])

# 计算变换矩阵
transformation_matrix = find_transformation_matrix(src_points, dst_points)

print("matrix:")
print(transformation_matrix)


