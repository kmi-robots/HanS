"""#Aggregate DL rankings on same anchor
    Given m rankings and n candidate objects, find a merged ranking
    The DL rankings are scored, so we can use an aggregated scoring function"""

import statistics

def merge_DL_ranks(r_list):

    # Given a list of m rankings of length n (no. of candidate objects)
    # returns a single ranking in the form (class, score)
    # by aggregating the scores class-wise
    all_class_scores ={}
    for i,r_ in enumerate(r_list):
        if i==0: #if first ranking also init class keys
            cls, _ = zip(*r_)
            all_class_scores = {k:[] for k in cls}
        for cl, score in r_:
            all_class_scores[cl].append(score)

    #Aggregate scores class-wise, here we take the average
    merged_rank = []
    for cl, slist in all_class_scores.items():
        merged_rank.append((cl, statistics.mean(slist)))

    # reorder by decreasing confidence score
    merged_rank = sorted(merged_rank, key=lambda x: (x[1],x[1]), reverse=True)

    return merged_rank