use crate::CONNECTION_MATRIX_SIZE;

pub(crate) struct RelayManager<const CONNECTION_MATRIX_SIZE: usize> {
    connection_matrix: [[u8; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
}

impl<const CONNECTION_MATRIX_SIZE: usize> RelayManager<CONNECTION_MATRIX_SIZE> {
    pub(crate) fn new() -> Self {
        RelayManager {
            connection_matrix: [[0; CONNECTION_MATRIX_SIZE]; CONNECTION_MATRIX_SIZE],
        }
    }
}
