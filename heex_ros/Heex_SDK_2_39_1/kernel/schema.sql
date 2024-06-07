CREATE TABLE IF NOT EXISTS jobs (
    event_uuid CHAR(256) NOT NULL,
    service_type INTEGER NOT NULL,
    state INTEGER NOT NULL,
    priority INTEGER NOT NULL,
    trigger_uuid CHAR(256) NOT NULL,
    created_at CHAR(256) NOT NULL,
    attempts INTEGER NOT NULL,
    filepath TEXT NOT NULL,
    PRIMARY KEY (event_uuid, service_type)
);
