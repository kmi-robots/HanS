import numpy as np
from params import get_parser
from pycoral.utils.dataset import read_label_file

from postgresql.size_queries import retrieve_dims, find_frontbox_shape

area_labels = ['XS','small','medium','large','XL']
depth_labels = ['flat','thin','thick','bulky']

parser = get_parser()
args_dict, unknown = parser.parse_known_args()
target_classes = read_label_file(args_dict.classes)
class_map = dict((v, k) for k, v in target_classes.items())  # swap keys with indices

lam = args_dict.Lambda
T = args_dict.T
w0 = args_dict.w0


def size_validate(input_r, cur, anchor_key, sizebase):
    """Given an input ranking,
            retains only the classes that are valid wrt to their size"""
    global class_map
    estimated_dims = retrieve_dims(cur, anchor_key)
    frontbox_shape = find_frontbox_shape(cur, anchor_key, estimated_dims[:2])
    sres = find_size_candidates(estimated_dims, sizebase, frontbox_shape)
    candidates_num, candidates_num_flat, candidates_num_thin, candidates_num_flatAR, candidates_num_thinAR = sres
    #Filter input ranking based on size candidates
    valid_rank_area = [(cid, score) for cid, score in input_r if cid in candidates_num]
    valid_rank_flat = [(cid, score) for cid, score in input_r if cid in candidates_num_flat]
    valid_rank_thin = [(cid, score) for cid, score in input_r if cid in candidates_num_thin]
    valid_rank_flatAR = [(cid, score) for cid, score in input_r if cid in candidates_num_flatAR]
    valid_rank_thinAR = [(cid, score) for cid, score in input_r if cid in candidates_num_thinAR]

    #Print results divided by size property but return only
    # the ranking where all three size properties (area,thickness,AR) are considered
    read_input = [(target_classes[cid], score) for cid, score in input_r]
    read_rank_area = [(target_classes[cid], score) for cid, score in valid_rank_area]
    read_rank_flat = [(target_classes[cid], score) for cid, score in valid_rank_flat]
    read_rank_thin = [(target_classes[cid], score) for cid, score in valid_rank_thin]
    read_rank_flatAR = [(target_classes[cid], score) for cid, score in valid_rank_flatAR]
    read_rank_thinAR = [(target_classes[cid], score) for cid, score in valid_rank_thinAR]

    print("Original top-5 ranking:")
    print(read_input[:5])

    print("Area top-5 ranking:")
    print(read_rank_area[:5])

    print("Flat/non-flat top-5 ranking:")
    print(read_rank_flat[:5])

    print("Thickness top-5 ranking:")
    print(read_rank_thin[:5])

    print("Flat+AR top-5 ranking:")
    print(read_rank_flatAR[:5])

    print("Thick+AR top-5 ranking:")
    print(read_rank_thinAR[:5])

    return valid_rank_thinAR


def find_size_candidates(estimated_dims, KB, box_shape):

    """Returns candidate classes based on the estimated sizes"""
    global class_map
    qual, is_flat, aspect_ratio, thinness = quantize(estimated_dims, box_shape)

    if is_flat: flat='flat'
    else: flat='non-flat'
    print("Object is '{}', '{}', '{}', '{}' ".format(qual,flat,thinness,aspect_ratio))

    """ Hybrid (area) """
    candidates = [oname for oname in KB.keys() if qual in str(
        KB[oname]["has_size"])]  # len([s for s in self.KB[oname]["has_size"] if s.startswith(qual)])>0]
    candidates_num = [class_map[oname.replace(' ', '_')] for oname in candidates]

    """ Hybrid (area + flat) """
    candidates_flat = [oname for oname in candidates if str(is_flat) in str(KB[oname]["is_flat"])]
    candidates_num_flat = [class_map[oname.replace(' ', '_')] for oname in candidates_flat]

    """ Hybrid (area + thin) """
    try:
        candidates_thin = [oname for oname in candidates if thinness in str(KB[oname]["thinness"])]
    except KeyError:  # annotation format variation
        candidates_thin = [oname for oname in candidates if thinness in str(KB[oname]["has_size"])]

    candidates_num_thin = [class_map[oname.replace(' ', '_')] for oname in candidates_thin]

    """ Hybrid (area + flat+AR) """
    candidates_flat_AR = [oname for oname in candidates_flat if aspect_ratio in str(KB[oname]["aspect_ratio"])]
    candidates_num_flatAR = [class_map[oname.replace(' ', '_')] for oname in candidates_flat_AR]

    """ Hybrid (area + thin +AR) """
    candidates_thin_AR = [oname for oname in candidates_thin if aspect_ratio in str(KB[oname]["aspect_ratio"])]
    candidates_num_thinAR = [class_map[oname.replace(' ', '_')] for oname in candidates_thin_AR]

    return [candidates_num, candidates_num_flat, candidates_num_thin, candidates_num_flatAR, candidates_num_thinAR]


def quantize(estimated_dims, front_hw):
    depth = min(estimated_dims)  # bc KB is for all three configurations of d1,d2,d3 here we make the assumption of considering only one configuration
    estimated_dims.remove(depth)  # i.e., the one where the min of the three is taken as depth
    d1, d2 = estimated_dims

    """ size quantization: from quantitative dims to qualitative labels"""
    qual = quant_size_qual(d1, d2, thresholds=T)
    flat = quant_flat(depth, len_thresh=lam[0])
    flat_flag = 'flat' if flat else 'non flat'
    # Aspect ratio based on crop
    aspect_ratio = quant_AR(front_hw, w0=w0)
    thinness = quant_thinness(depth, cuts=lam)
    # cluster = qual + "-" + thinness

    # print("Detected size is %s" % qual)
    # print("Object is %s" % flat_flag)
    # print("Object is %s" % aspect_ratio)
    # print("Object is %s" % thinness)
    return qual,flat,aspect_ratio,thinness

def quant_size_qual(dim1, dim2,thresholds=[]):

    estimated_area = np.log(dim1 * dim2)
    if estimated_area < thresholds[0]: return 'XS'
    elif estimated_area >= thresholds[-1]: return 'XL'
    else: #intermediate cases
        for i in range(len(thresholds)-1):
            if (estimated_area>=thresholds[i] and estimated_area < thresholds[i+1]):
                return area_labels[i+1]

def quant_flat(depth, len_thresh = 0.0): #if depth greater than x% of its min dim then non flat
    depth = np.log(depth)
    if depth <= len_thresh: return True
    else: return False

def quant_thinness(depth, cuts=[]):
    """
    Rates object thinness/thickness based on measured depth
    """
    depth = np.log(depth)
    if depth <= cuts[0]: return 'flat'
    elif depth > cuts[-1]: return 'bulky'
    else: # intermediate cases
        for i in range(len(cuts)-1):
            if depth > cuts[i] and depth <= cuts[i+1]:
                return depth_labels[i+1]

def quant_AR(crop_dims,w0=1.4):
    """
    Returns aspect ration based on 2D crop dimensions
    and estimated dimensions
    """
    height, width = crop_dims #used to derive the orientation
    # print("crop dimensions are %s x %s" % (str(width), str(height)))
    if height >= width:
        #h = max(d1,d2)
        #w = min(d1,d2)
        AR = height/width
        if AR >= w0: return 'TTW'
        else: return 'EQ' #h and w are comparable
    if height < width:
        #h = min(d1, d2)
        #w = max(d1, d2)
        AR = width/height
        if AR >= w0: return 'WTT'
        else: return 'EQ' #h and w are comparable