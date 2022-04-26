##Creating trigger functions

From PGAdmin, click on database name > trigger functions > create > trigger function.
The code below is to be added to the "Code" tab.

We will first create a trigger 'assign_anchor', that 
decides whether to assign a new measurements (i.e., new records in table measurements) to an existing anchor or to create a new anchor.
The trigger looks for anchors that have a `location_3d` close to the `centroid_3d` of the measurement. The closest one is considered the most promising candidate, and it is used as the anchor for the measurement.
If this approach fails, it means this is the first measurement related to that specific physical object. Therefore, the trigger creates a new anchor initialized with the parameters of the measurement.


```sql
BEGIN
	INSERT INTO measurements_anchors(anchor_key, object_key)
	SELECT anchor_key, NEW.object_key
	FROM anchors
	WHERE ST_3DDistance(NEW.centroid_3d, location_3d) < 0.2
	ORDER BY ST_3DDistance(NEW.centroid_3d, location_3d)
	LIMIT 1;
	
	IF NOT EXISTS (SELECT * 
				   FROM measurements_anchors
				   WHERE object_key = NEW.object_key) THEN
		INSERT INTO anchors(convex_hull_union, location_3d, last_update, label, robot_position, complete)
		VALUES (NEW.convex_hull, NEW.centroid_3d, NEW.stamp, NEW.label, NEW.robot_position, FALSE);
        INSERT INTO measurements_anchors(anchor_key, object_key)
		SELECT anchor_key, NEW.object_key
		FROM anchors
		WHERE location_3d = NEW.centroid_3d AND last_update = NEW.stamp
		LIMIT 1;
	END IF;
	RETURN NEW;
END;
```

We then create a second trigger named 'update_anchor' that actually updates the content of the anchor, based on the new measurements acquired.
This trigger is activated after the previous one, since it is connected to an insertion in the `measurements_anchors` table. 
A new entry in the table means a new measurement has been generated and assigned to an anchor, therefore the target anchor must be updated. 
The mean of the robot positions across measurements is also considered as robot position for the anchor. 
When a new anchor is created or a new measurement is found for an existing anchor, the field complete is set to False.
In this way, the object anchor will be later selected for reasoning. 

```sql
BEGIN
    UPDATE anchors
    SET last_update = (SELECT stamp
                       FROM measurements
                       WHERE object_key = NEW.object_key),
        
        robot_position = (SELECT ST_GeometricMedian(ST_Collect(m.robot_position, a.robot_position))
                        FROM measurements as m, anchors as a 
                        WHERE m.object_key = NEW.object_key
                        AND a.anchor_key = NEW.anchor_key),
        complete = FALSE
    WHERE anchor_key = NEW.anchor_key;
    UPDATE anchors
    SET location_3d =(SELECT ST_GeometricMedian(ST_Points(convex_hull_union))
                     FROM anchors
                    WHERE anchors.anchor_key = NEW.anchor_key) 
    WHERE anchor_key = NEW.anchor_key;
    RETURN NEW;
END;
```
##Adding triggers to a table
From PGAdmin, right click on the measurements table and select Create > Trigger. 
In the definition tab select the assign_anchor function created below. 
Under Events, select the AFTER INSERT option. 

Then, repeat the same steps to add the update_anchor function to the measurements_anchors table.
The setting AFTER INSERT is the same in this case. 