"""Spatial operations on postgresql db"""

def populate_with_boxes(connection,cursor, sf=1.2):
    """Creation of min oriented bounding box, contextualised bounding box
    and six halfspaces, for each object/spatialRegion
    """

    # for all object anchors
    # Find min oriented 3D box bounding of polyhedral surface
    cursor.execute('SELECT anchor_key, oriented_envelope, ST_ZMin(convex_hull_union),'
                   ' ST_ZMax(convex_hull_union) FROM anchors')

    query_res = [(str(r[0]), str(r[1]), float(r[2]), float(r[3])) for r in cursor.fetchall()]

    for id_, envelope, zmin, zmax in query_res:

        height = zmax - zmin
        # ST_Translate is needed here because the oriented envelope is projected on XY so we also need to translate
        # everything up by the original height after extruding in this case
        up1_mask = 'UPDATE anchors SET bbox = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s)' \
                   ' WHERE object_key = %s;'
        cursor.execute(up1_mask, (envelope, str(height), zmin, id_))
        connection.commit()

        #Derive CBB
        #minimum of the angles between:
        #1) line connecting robot position and object centroid
        #2) line extending each of the edges of the oriented base of the object


        cursor.execute('SELECT ST_Angle(ST_MakeLine(robot_position, ST_Centroid(oriented_envelope)), ' \
            'ST_MakeLine(ST_PointN(ST_ExteriorRing(oriented_envelope), 1), ' \
            'ST_PointN(ST_ExteriorRing(oriented_envelope)), 2))), ' \
            'ST_Angle(ST_MakeLine(robot_position, ST_Centroid(oriented_envelope)), ' \
            'ST_MakeLine(ST_PointN(ST_ExteriorRing(oriented_envelope), 2), ' \
            'ST_PointN(ST_ExteriorRing(oriented_envelope), 3))), ' \
            'ST_Angle(ST_MakeLine(robot_position, ST_Centroid(oriented_envelope)), ' \
            'ST_MakeLine(ST_PointN(ST_ExteriorRing(oriented_envelope), 3), ' \
            'ST_PointN(ST_ExteriorRing(oriented_envelope), 4))), ' \
            'ST_Angle(ST_MakeLine(robot_position, ST_Centroid(oriented_envelope)), ' \
            'ST_MakeLine(ST_PointN(ST_ExteriorRing(oriented_envelope), 4), ' \
            'ST_PointN(ST_ExteriorRing(oriented_envelope), 1)))'\
            ' FROM anchors ' \
            ' WHERE anchor_key = \'' + id_ + '\';')

        connection.commit()

        angles = list(cursor.fetchone())
        angle = min(angles)

        up2_mask = 'UPDATE anchors SET cbb = ST_Rotate(bbox, %s,' \
                   ' ST_Centroid(oriented_envelope))' \
                   ' WHERE anchor_key = %s;'
        cursor.execute(up2_mask, (str(angle), id_))
        connection.commit()

        # Derive the six halfspaces, based on scaling factor sf

        # top and bottom ones based on MinOriented, i.e., extruded again from oriented envelope
        up_topbtm = 'UPDATE anchors SET  tophsproj = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s),'\
                    ' bottomhsproj = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s)'\
                   ' WHERE anchor_key = %s;'
        cursor.execute(up_topbtm, (envelope, str(height*sf), zmax, envelope, str(height * sf), zmin-height*sf, id_))

        # Identify the base of the CBB, i.e., oriented envelope of points with z=zmin
        # Define 4 rectangles from the base and expand 2D
        q_hs = 'SELECT St_Rotate(hsX, alpha, St_centroid(aligned_base)), St_Rotate(hsY, alpha, St_centroid(aligned_base)), base, robot_position '\
                'FROM('\
                    'SELECT (St_Dump(r1)).geom as hsX, (St_Dump(r2)).geom as hsY, alpha, aligned_base, base, w, d, robot_position '\
                    'FROM('\
                    'SELECT robot_position, alpha, aligned_base, base, w, d, St_Difference(St_Expand(aligned_base, %s * w, 0),'\
                        'St_Scale(aligned_base, St_MakePoint(1.00001,1.00001), St_Centroid(aligned_base))) as r1,'\
                        'St_Difference(St_Expand(aligned_base, 0, %s * d), St_Scale(aligned_base,'\
                        'St_MakePoint(1.00001,1.00001), St_Centroid(aligned_base))) as r2 '\
                        'FROM( SELECT St_Rotate(base,-alpha, ST_Centroid(base)) as aligned_base, w, d, alpha, base, robot_position '\
                        'FROM( SELECT robot_position,base, St_XMax(base) - St_XMin(base) as w, St_YMax(base) - St_YMin(base) as d,'\
                        'St_Angle(St_MakeLine(ST_PointN(ST_ExteriorRing(base),1), ST_PointN(ST_ExteriorRing(base),2)),'\
		                'St_MakeLine(robot_position, ST_MakePoint(ST_X(robot_position),ST_Y(robot_position)+ 1))) as alpha '\
		                'FROM (SELECT ST_OrientedEnvelope(St_Collect((dbox).geom)) as base, robot_position '\
	  					'FROM(SELECT St_DumpPoints(cbb) as dbox, '\
                                'St_ZMin(cbb) as zmin, robot_position FROM anchors '\
		    					'WHERE anchor_key=%s) as dt '\
	                            'WHERE St_Z((dbox).geom) = zmin '\
                                'GROUP BY robot_position )  as basal '\
                        ') as angles'\
                        ') as aligned'\
                        ') as hs'\
                ')as fcheck'

        cursor.execute(q_hs, (str(sf),str(sf), id_))
        q_res = cursor.fetchall() # for each object, 2 rows by 3 colums (i.e., 4 halfspaces + base of cbb repeated twice)

        # Interpret what is L/R/front/back among those boxes
        all_dis =[]
        all_angles = []
        all_hss=[]
        for q in q_res:
            all_hss.append(q[0])
            all_hss.append(q[1])

        for lr, fb, base,rp in q_res:

            qdis = 'SELECT St_Distance(%s, St_Centroid(St_GeomFromEWKT(%s)))'
            cursor.execute(qdis, (rp, lr,))
            qdsir = cursor.fetchone()
            qdisr2 = qdsir[0]
            all_dis.append(qdisr2)

            # Distance between robot position and hs centroid
            cursor.execute(qdis, (rp,fb,))
            qdisr = cursor.fetchone()[0]
            all_dis.append(qdisr)

            # angle between robot position and base centroid (St_Angle is computed clockwise)
            qang = 'SELECT St_Angle(St_MakeLine(%s,St_Centroid(%s)), St_MakeLine(%s '\
                        ',St_Centroid(St_GeomFromEWKT(%s))))'
            cursor.execute(qang, (rp,base,rp,lr))
            qangr = cursor.fetchone()[0]
            all_angles.append(qangr)

            cursor.execute(qang, (rp, base, rp, fb))
            qangr = cursor.fetchone()[0]
            all_angles.append(qangr)

        #front is the nearest one to robot position
        front_idx = all_dis.index(min(all_dis))
        fronths = all_hss[front_idx]
        #Is the index found for fronths odd or even? Take other hs along same axis as back hs
        if front_idx % 2 == 0:
            back_idx = [indd for indd in range(4) if indd %2==0 and indd!=front_idx][0] #other even index that is not the fron one
        else:
            back_idx = [indd for indd in range(4) if indd %2!=0 and indd!=front_idx][0] #other even index that is not the fron one

        backhs = all_hss[back_idx]

        #Left one has the biggest angle with robot position and base centroid

        #remove front and back from angle list as the closest one may be close to 360 degrees, i.e., aligned with robot's position
        old_ids = [az for az, a in enumerate(all_angles) if az not in [front_idx, back_idx]]
        all_angles = [a for az, a in enumerate(all_angles) if az not in [front_idx,back_idx]]

        left_idx = all_angles.index(max(all_angles))
        left_idx = old_ids[left_idx]
        lefths = all_hss[left_idx]

        #right hs then is the only remaining index
        right_idx = [indd for indd in range(4) if indd not in [front_idx,back_idx,left_idx]][0]
        righths = all_hss[right_idx]

        # Extrude + Translate 3D & update table with halfspace columns
        up_others = 'UPDATE anchors SET  lefthsproj = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s),' \
                    ' righthsproj = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s),' \
                    ' fronthsproj = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s),' \
                    ' backhsproj = ST_Translate(ST_Extrude(%s, 0, 0, %s), 0, 0, %s)' \
                    ' WHERE anchor_key = %s;'
        cursor.execute(up_others,
                                (lefths, str(height), zmin, righths, str(height), zmin,
                                 fronths,str(height), zmin, backhs,str(height), zmin, id_))
        connection.commit()