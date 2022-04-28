CREATE TABLE IF NOT EXISTS measurements(
        object_key serial PRIMARY KEY,
        stamp timestamp NOT NULL,
        convex_hull geometry,
        tracking_id varchar,
        label varchar,
        centroid_3d geometry,
        robot_position geometry);


CREATE TABLE IF NOT EXISTS anchors(
    anchor_key serial PRIMARY KEY,
    location_3d geometry NOT NULL,
    complete boolean,
    last_update timestamp NOT NULL,
    label varchar,
    convex_hull_union geometry,
    oriented_envelope geometry,
    bbox geometry,
    cbb geometry,
    bottomhsproj geometry,
    tophsproj geometry,
    lefthsproj geometry,
    righthsproj geometry,
    backhsproj geometry,
    fronthsproj geometry,
    d1 double precision,
    d2 double precision,
    d3 double precision,
    robot_position geometry
    );

CREATE TABLE IF NOT EXISTS measurements_anchors (
    object_key integer,
    anchor_key integer,
    primary key (object_key, anchor_key),
    FOREIGN KEY (object_key) REFERENCES measurements (object_key),
    FOREIGN KEY (anchor_key) REFERENCES anchors (anchor_key));

CREATE TABLE IF NOT EXISTS vg_rels(
    relation_id serial PRIMARY KEY,
    predicate_name varchar NOT NULL,
    subject_polygon geometry,
    object_top_projection geometry);

CREATE TABLE IF NOT EXISTS sw_walls(
        id serial PRIMARY KEY,
        surface geometry NOT NULL);