import sys

def main():

    # TODO
    # connect to db
    # select all anchors marked as not complete
    # union of convex hull in same anchor
    # retrieve all DL predictions related to anchor
    # compute size and spatial bboxes of union chull
    # compute QSR between anchors
    # aggregate DL with/without reasoning + meta-reasoning options

    # TODO (not here) code to prep size and spatial background knowledge given a set of classes as input
    # TODO (not here) add code for wall annotation given a map

    # expand QSR graph based on Quasimodo concepts ./data/commonsense_extracted.json
    # check rules
    # once done with reasoning, mark all object anchors as complete
    # evaluate results
    pass


if __name__ == "__main__":
    main()
    sys.exit(0)