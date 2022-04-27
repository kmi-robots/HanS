"""To extract synsets automatically given any list of class names
Be sure to check output for accuracy as terms alone are not sufficient to identify the most relevant synset
"""

from nltk.corpus import wordnet as wn
import json
from pattern.text.en import singularize


def map_to_synsets(class_list, args):

    syn_dict = {k:[] for k in class_list}
    for cname in class_list:
        if cname!='glass':
            cname_sing = singularize(cname)
        else: cname_sing = cname
        print(cname_sing)

        syns = [syn.name() for syn in wn.synsets(cname_sing, pos=wn.NOUN)] # only synsets of noun part of speech

        if len(syns) == 0: #try with sub keyword (last one) - e.g., extinguisher, sign, screen
            kw = cname_sing.split('_')[-1]
            syns = [syn.name() for syn in wn.synsets(kw, pos=wn.NOUN)]

        if len(syns)<=1:
            syn_dict[cname].extend(syns)
        else: syn_dict[cname].extend(syns[:1]) #only consider top-1 synset, most relevant


    with open(args.syn_path, 'w') as fout:
        json.dump(syn_dict, fout)

    return syn_dict