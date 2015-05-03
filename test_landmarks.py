from landmarks import *

lm = landmark()
lm.pos = [1,1]
add_to_DB(lm)
landmarkDB[0].total_times_observed = 20
landmarkDB[0].id_ = 10

lm2 = landmark()
lm2.pos = [2,2]
lm2.total_times_observed = 16
lm2.id_ = 11

print(get_closest_association(lm2).id_)
