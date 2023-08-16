pub type Result<T, E = Error> = std::result::Result<T, E>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("CDR error: {0}")]
    Cdr(#[from] cdr::Error),

    #[error("{0}")]
    Other(#[from] Box<dyn std::error::Error + Sync + Send + 'static>),

    #[error("The sensor with ID {sensor_id} is ownerless.")]
    OwnerlessSensor { sensor_id: u32 },

    #[error("The vehicle is NPC")]
    Npc { npc_role_name: String },
}
