pub type Result<T, E = Error> = std::result::Result<T, E>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("CDR error: {0}")]
    Cdr(#[from] cdr::Error),

    #[error("Communication error: {0}")]
    Communication(&'static str),

    #[error("The sensor with ID {sensor_id} is ownerless")]
    OwnerlessSensor { sensor_id: u32 },

    #[error("The vehicle is NPC")]
    Npc { npc_role_name: String },

    #[error("The issue is from Carla: {0}")]
    CarlaIssue(&'static str),

    #[error("{0}")]
    Other(#[from] Box<dyn std::error::Error + Sync + Send + 'static>),
}
