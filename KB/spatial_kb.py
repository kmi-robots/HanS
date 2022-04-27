""" Code to prep spatial background knowledge --- no filter by specific class at this stage"""

import json
from re import search
from collections import Counter

from postgresql.spatial_queries import add_VG_row,compute_spatial_op

predicate_set = [("in", "inside"), ("on",),  ("on top of",), ("against",),
                              ("front",), ("behind",), ("left",), ("right",),
                              ("next to","beside","adjacent", "on side of"),  ("under", "below"),
                              ("above",), ("near","by","around")] #predicate pool, with synonyms, needed to derive the commonsense predicates of (Landau & Jackendoff 1993)
                                                    # also adopted in our paper

def extract_spatial_kb(conn, cur, args):

    # Load VG raw relationships and aliases
    raw_data = load_rel_bank(args)  # , alias_index
    # flatten alias index
    # alias_mergedlist = list(alias_index.values())

    VG_stats = {k: {} for k in ["predicates", "subjects", "objects"]}
    for entry in raw_data:  # entry = image
        img_rels = entry["relationships"]  # all relationships for a given image

        for rel in img_rels:
            pred = rel['predicate'].lower()
            sub_syn = rel['subject']['synsets']
            obj_syn = rel['object']['synsets']

            if len(sub_syn) != 1 or len(obj_syn) != 1 or pred == '' or pred == ' ' \
                    or pred == 'of' or pred == 'to':
                # Skipping:
                # (ii) relations without both sub and object synsets
                # (iii) compound periods, i.e., more than one synset per entity
                # e.g.  subject: green trees seen  pred: green trees by road object: trees on roadside.)
                # e.g.  subject: see cupboard  pred: cupboard black object: cupboard not white. )
                # (iv) as well as empty predicates
                # (v) "of" and "to" are too generic and match "next to" and "on top of" ambiguously
                continue

            # Find closest match between pred and target predicate set, if any
            hits = [(p_, search(pred + " ", p_ + " ").end() - search(pred + " ", p_ + " ").start()) for p in
                    predicate_set for p_ in p if search(pred + " ", p_ + " ") is not None]
            # + " " #e.g., extra space to avoid that 'on' appears as part of 'front'
            if len(hits) == 0:
                print("Predicate %s not similar to any in the list" % pred)
                continue  # otherwise skip

            # Find most exact/relevant match
            span = 0
            for hit, l in hits:
                if hit == pred:  # prioritise exact match, if any
                    match = hit
                    break
                # otherwise based on length of hit span
                if l > span:
                    span = l
                    match = hit

            if match == 'left':
                VG_stats = update_VG_stats(VG_stats, "leftOf", sub_syn, obj_syn)
                # union of right and left also counts as beside
                VG_stats = update_VG_stats(VG_stats, "beside", sub_syn, obj_syn)
            elif match == 'right':
                VG_stats = update_VG_stats(VG_stats, "rightOf", sub_syn, obj_syn)
                # union of right and left also counts as beside
                VG_stats = update_VG_stats(VG_stats, "beside", sub_syn, obj_syn)
            elif match == 'front':
                VG_stats = update_VG_stats(VG_stats, "inFrontOf", sub_syn, obj_syn)
            elif match == 'behind' or match == 'above' or match == 'below' or match == 'in' \
                    or match == 'against' or match == 'beside' or match == 'near':
                VG_stats = update_VG_stats(VG_stats, match, sub_syn, obj_syn)  # no need to reformat QSR name
            elif match == 'under':
                VG_stats = update_VG_stats(VG_stats, "below", sub_syn, obj_syn)
            elif match == 'on top of':
                VG_stats = update_VG_stats(VG_stats, "onTopOf", sub_syn, obj_syn)
            elif match == "inside":
                VG_stats = update_VG_stats(VG_stats, "in", sub_syn, obj_syn)
            elif match in ("next to", "adjacent", "on side of"):
                VG_stats = update_VG_stats(VG_stats, "beside", sub_syn, obj_syn)
            elif match in ("by", "around"):
                VG_stats = update_VG_stats(VG_stats, "near", sub_syn, obj_syn)

            elif match == 'on':
                # Disambiguate on top of from against (i.e., seen as union of leanson and affixed on)
                # 1. Populate spatial DB #
                # only used to check, through PostGIS, whether object is on top of another or not
                rel_id = rel['relationship_id']  # to use as primary key

                # 2D Bounding box corners of subject (e.g., "cup ON table" subj = cup, obj = table)
                # Format: from top-left corner anti-clockwise
                # PostGIS requires to repeat top-left twice to close the ring
                x1, y1 = rel['subject']['x'], rel['subject']['y']
                x2, y2 = x1, (y1 + rel['subject']['h'])
                x3, y3 = (x1 + rel['subject']['w']), (y1 + rel['subject']['h'])
                x4, y4 = (x1 + rel['subject']['w']), y1
                sub_coords = ((x1, y1), (x2, y2), (x3, y3), (x4, y4), (x1, y1))

                # top 2D half-space projection of object bbox
                x1, y1 = rel['object']['x'], (rel['object']['y'] - rel['object']['h'])
                x2, y2 = rel['object']['x'], rel['object']['y']
                x3, y3 = (x1 + rel['object']['w']), rel['object']['y']
                x4, y4 = (x1 + rel['object']['w']), (rel['object']['y'] - rel['object']['h'])
                obj_top_proj_coords = ((x1, y1), (x2, y2), (x3, y3), (x4, y4), (x1, y1))

                add_VG_row(cur, [rel_id, pred, sub_coords, obj_top_proj_coords])
                conn.commit()

                overlaps, touches = compute_spatial_op(cur, rel_id)
                if overlaps or touches:  # if the subject overlaps or touches the top hs projection of object
                    # then on top of case
                    match = 'onTopOf'
                else:  # against cases  #we see it as the union of affixed on and lean on
                    match = 'against'
                VG_stats = update_VG_stats(VG_stats, match, sub_syn, obj_syn)

            # Plus, all spatial rels except 'above' count as 'near'
            if match != 'above': VG_stats = update_VG_stats(VG_stats, 'near', sub_syn, obj_syn)

    # save VG stats locally
    with open(args.spatialkb_path, 'w') as fout:
        json.dump(VG_stats, fout)

    return VG_stats


def load_rel_bank(args):
    with open(args.vg_src) as ind:
        raw_data = json.load(ind)
    return raw_data


def update_VG_stats(stats_dict,pred,sub_syn, obj_syn):

    stats_dict = add_relation_counts(stats_dict,pred,sub_syn, obj_syn)
    # Repeat all of the above for the case of near, i.e., all predicates generalise back to near
    #if pred not in alias_index['near']:
    #    stats_dict = add_relation_counts(stats_dict, "near", alias_index["near"], sub_syn, obj_syn)
    return stats_dict

def add_relation_counts(stats_dict,pred,sub_syn, obj_syn):
    # How many times subj - pred - obj?
    if pred not in stats_dict["predicates"]:
        stats_dict["predicates"][pred] = {}
        stats_dict["predicates"][pred]["relations"] = Counter()
        #stats_dict["predicates"][pred]["aliases"] = aliases
    stats_dict["predicates"][pred]["relations"][str((str(sub_syn[0]), str(obj_syn[0])))] += 1

    # how many times subject in relationship of type pred?
    if str(sub_syn[0]) not in stats_dict["subjects"]:
        stats_dict["subjects"][str(sub_syn[0])] = Counter()
    stats_dict["subjects"][str(sub_syn[0])][pred] += 1

    # how many times object in relationship of type pred?
    if str(obj_syn[0]) not in stats_dict["objects"]:
        stats_dict["objects"][str(obj_syn[0])] = Counter()
    stats_dict["objects"][str(obj_syn[0])][pred] += 1
    return stats_dict




