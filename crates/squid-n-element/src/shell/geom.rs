//! 要素の幾何量。
//!
//! - [`element_area`] — 四辺形を2三角形に分割した面積

pub(crate) fn element_area(coords: &[[f64; 3]; 4]) -> f64 {
    // Area of quadrilateral as sum of two triangles
    let v01 = [
        coords[1][0] - coords[0][0],
        coords[1][1] - coords[0][1],
        coords[1][2] - coords[0][2],
    ];
    let v02 = [
        coords[2][0] - coords[0][0],
        coords[2][1] - coords[0][1],
        coords[2][2] - coords[0][2],
    ];
    let v12 = [
        coords[2][0] - coords[1][0],
        coords[2][1] - coords[1][1],
        coords[2][2] - coords[1][2],
    ];
    let v13 = [
        coords[3][0] - coords[1][0],
        coords[3][1] - coords[1][1],
        coords[3][2] - coords[1][2],
    ];

    let cross012 = [
        v01[1] * v02[2] - v01[2] * v02[1],
        v01[2] * v02[0] - v01[0] * v02[2],
        v01[0] * v02[1] - v01[1] * v02[0],
    ];
    let area012 = 0.5
        * (cross012[0] * cross012[0] + cross012[1] * cross012[1] + cross012[2] * cross012[2])
            .sqrt();

    // Using triangles 0-1-2 and 1-2-3
    let cross123 = [
        v12[1] * v13[2] - v12[2] * v13[1],
        v12[2] * v13[0] - v12[0] * v13[2],
        v12[0] * v13[1] - v12[1] * v13[0],
    ];
    let area123 = 0.5
        * (cross123[0] * cross123[0] + cross123[1] * cross123[1] + cross123[2] * cross123[2])
            .sqrt();

    area012 + area123
}
