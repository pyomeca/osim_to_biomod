import numpy as np


def transform_polygon_to_triangles(polygons, nodes, normals) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform any polygons with more than 3 edges into polygons with 3 edges (triangles)."""

    if polygons.shape[1] == 3:
        return polygons, nodes, normals

    elif polygons.shape[1] == 4:
        return convert_quadrangles_to_triangles(polygons, nodes, normals)

    elif polygons.shape[1] > 4:
        return convert_polygon_to_triangles(polygons, nodes, normals)

    else:
        RuntimeError("The polygons array must have at least 3 columns.")


def norm2(v):
    """Compute the squared norm of each row of the matrix v."""
    return np.sum(v**2, axis=1)


def convert_quadrangles_to_triangles(polygons, nodes, normals) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform polygons with 4 edges (quadrangles) into polygons with 3 edges (triangles)."""
    # 1. Search for quadrangles
    quadrangles_idx = np.where((polygons[:, 3] != 0) & (~np.isnan(polygons[:, 3])))[0]
    triangles_idx = np.where(np.isnan(polygons[:, 3]))[0]

    # transform polygons[quadrangles, X] as a list of int
    polygons_0 = polygons[quadrangles_idx, 0].astype(int)
    polygons_1 = polygons[quadrangles_idx, 1].astype(int)
    polygons_2 = polygons[quadrangles_idx, 2].astype(int)
    polygons_3 = polygons[quadrangles_idx, 3].astype(int)

    # 2. Determine triangles to be made
    mH = 0.5 * (nodes[polygons_0] + nodes[polygons_2])  # Barycentres AC
    mK = 0.5 * (nodes[polygons_1] + nodes[polygons_3])  # Barycentres BD
    KH = mH - mK
    AC = -nodes[polygons_0] + nodes[polygons_2]  # Vector AC
    BD = -nodes[polygons_1] + nodes[polygons_3]  # Vector BD
    # Search for the optimal segment for the quadrangle cut
    type_ = np.sign((np.sum(KH * BD, axis=1) / norm2(BD)) ** 2 - (np.sum(KH * AC, axis=1) / norm2(AC)) ** 2)

    # 3. Creation of new triangles
    tBD = np.where(type_ >= 0)[0]
    tAC = np.where(type_ < 0)[0]
    # For BD
    PBD_1 = np.column_stack(
        [polygons[quadrangles_idx[tBD], 0], polygons[quadrangles_idx[tBD], 1], polygons[quadrangles_idx[tBD], 3]]
    )
    PBD_2 = np.column_stack(
        [polygons[quadrangles_idx[tBD], 1], polygons[quadrangles_idx[tBD], 2], polygons[quadrangles_idx[tBD], 3]]
    )
    # For AC
    PAC_1 = np.column_stack(
        [polygons[quadrangles_idx[tAC], 0], polygons[quadrangles_idx[tAC], 1], polygons[quadrangles_idx[tAC], 2]]
    )
    PAC_2 = np.column_stack(
        [polygons[quadrangles_idx[tAC], 2], polygons[quadrangles_idx[tAC], 3], polygons[quadrangles_idx[tAC], 0]]
    )

    # 4. Matrix of final polygons
    new_polygons = np.vstack([polygons[triangles_idx, :3], PBD_1, PBD_2, PAC_1, PAC_2])

    # # 5. Duplicate normals
    # new_normals = np.vstack([normals[triangles_idx, 3],
    #                         normals[quadrangles_idx[tBD], 3],
    #                         normals[quadrangles_idx[tBD], 3],
    #                         normals[quadrangles_idx[tAC], 3],
    #                         normals[quadrangles_idx[tAC], 3]]).T

    return new_polygons, nodes, normals


def convert_pentagons_to_triangles(polygons, nodes):
    """
    Transform polygons with 5 edges (pentagons) into polygons with 3 edges (triangles).
    """
    # Search for pentagons
    pentagons = np.where((polygons[:, 4] != 0) & (~np.isnan(polygons[:, 4])))[0]

    # Split the pentagons into triangles
    # For simplicity, we'll use vertex 0 as the common vertex and form triangles:
    # (0, 1, 2), (0, 2, 3), and (0, 3, 4)
    T1 = np.column_stack([polygons[pentagons, 0], polygons[pentagons, 1], polygons[pentagons, 2]])
    T2 = np.column_stack([polygons[pentagons, 0], polygons[pentagons, 2], polygons[pentagons, 3]])
    T3 = np.column_stack([polygons[pentagons, 0], polygons[pentagons, 3], polygons[pentagons, 4]])

    # Return all the triangles
    return np.vstack([T1, T2, T3])


def convert_hexagons_to_triangles(polygons, nodes):
    """
    Transform polygons with 6 edges (hexagons) into polygons with 3 edges (triangles).
    """
    # Search for hexagons
    hexagons = np.where((polygons[:, 5] != 0) & (~np.isnan(polygons[:, 5])))[0]

    # Split the hexagons into triangles
    # For simplicity, we'll use vertex 0 as the common vertex and form triangles:
    # (0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5), (0, 5, 6)
    T1 = np.column_stack([polygons[hexagons, 0], polygons[hexagons, 1], polygons[hexagons, 2]])
    T2 = np.column_stack([polygons[hexagons, 0], polygons[hexagons, 2], polygons[hexagons, 3]])
    T3 = np.column_stack([polygons[hexagons, 0], polygons[hexagons, 3], polygons[hexagons, 4]])
    T4 = np.column_stack([polygons[hexagons, 0], polygons[hexagons, 4], polygons[hexagons, 5]])
    T5 = np.column_stack([polygons[hexagons, 0], polygons[hexagons, 5], polygons[hexagons, 6]])

    # Return all the triangles
    return np.vstack([T1, T2, T3, T4, T5])


def convert_septagons_to_triangles(polygons, nodes):
    """
    Transform polygons with 7 edges (septagons) into polygons with 3 edges (triangles).
    """
    # Search for septagons
    septagons = np.where((polygons[:, 6] != 0) & (~np.isnan(polygons[:, 6])))[0]

    # Split the septagons into triangles
    # For simplicity, we'll use vertex 0 as the common vertex and form triangles:
    # (0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5), (0, 5, 6), (0, 6, 7)
    T1 = np.column_stack([polygons[septagons, 0], polygons[septagons, 1], polygons[septagons, 2]])
    T2 = np.column_stack([polygons[septagons, 0], polygons[septagons, 2], polygons[septagons, 3]])
    T3 = np.column_stack([polygons[septagons, 0], polygons[septagons, 3], polygons[septagons, 4]])
    T4 = np.column_stack([polygons[septagons, 0], polygons[septagons, 4], polygons[septagons, 5]])
    T5 = np.column_stack([polygons[septagons, 0], polygons[septagons, 5], polygons[septagons, 6]])
    T6 = np.column_stack([polygons[septagons, 0], polygons[septagons, 6], polygons[septagons, 7]])

    return np.vstack([T1, T2, T3, T4, T5, T6])


def convert_polygon_to_triangles(polygons, nodes, normals) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Transform any polygons with more than 3 edges into polygons with 3 edges (triangles).
    """

    # Search for polygons with more than 3 edges
    polygons_with_more_than_3_edges = np.where((polygons[:, 3] != 0) & (~np.isnan(polygons[:, 3])))[0]
    polygons_with_3_edges = np.where(np.isnan(polygons[:, 3]))[0]

    triangles = []
    new_normals = []
    for j, poly_idx in enumerate(polygons_with_more_than_3_edges):
        # get only the non-nan values
        current_polygon = polygons[poly_idx, np.isnan(polygons[poly_idx]) == False]
        # current_normal = normals[poly_idx]
        # Split the polygons into triangles
        # For simplicity, we'll use vertex 0 as the common vertex and form triangles:
        # (0, 1, 2), (0, 2, 3), (0, 3, 4), ..., (0, n-2, n-1)

        for i in range(1, current_polygon.shape[0] - 1):
            triangles.append(np.column_stack([polygons[poly_idx, 0], polygons[poly_idx, i], polygons[poly_idx, i + 1]]))

        # # Duplicate normals
        # for i in range(1, current_polygon.shape[0] - 1):
        #     new_normals.append(
        #         np.column_stack(
        #             [current_normal[0],
        #              current_normal[1],
        #              current_normal[2],
        #     ]
        #         )
        #     )

    return (
        np.vstack([polygons[polygons_with_3_edges, :3], *triangles]),
        nodes,
        # np.vstack([normals[polygons_with_3_edges, :3], *new_normals]),
        normals,
    )
