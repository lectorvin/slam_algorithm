# robot position = [x,y,theta]
# laserdata = [2, 2, 2.1, 2.1, 2.1, 2.1, 2.2...]  - distance to objects
import math
import random


CONV = math.pi / 180.0
DEGREES_PER_SCAN = 0.5
MAX_RANGE = 1
MAX_LANDMARKS = 3000
MAX_TRIALS = 1000        # times to run algorithm
MAX_SAMPLE = 10          # numbers of randomly selected points
MAX_ERROR = 0.5          # 20cm (?) from line
MIN_OBSERVATIONS = 15    # landmarkd should be observated to be recognized as line
MIN_LINEPOINTS = 30      # points to recognize line
RANSAC_TOLERANCE = 0.05
RANSAC_CONSENSUS = 30
LIFE = 40



class landmark(object):
  """ landmark we try to find
      pos = (x, y);
      total_times_observed - total times we have saw this landmark;
  """

  def __init__(self):
    self.total_times_observed = 0   # the number of times we have seen landmark
    self.id_ = -1
    self.life = LIFE          # a life counter used to determine whether to discard a landmark
    self.pos = [0, 0]
    self.a = -1               # RANSAC: Now store equation of a line
    self.b = -1
    self.range_ = 0           # last observed range to landmark
    self.bearing = 0          # last observed bearing to landmark
    self.bearing_error = 0    # bearing from robot position to the wall we are using as a landmark
    self.range_error = 0      # distance from robot position to the wall we are using as a landmark


landmarkDB = [landmark() for i in range(MAX_LANDMARKS)]
DBSize = 0
EKFLandmarks = 0
IDtoID = [[0,0] for i in range(MAX_LANDMARKS)]
# int [,] = new int IDtoID [MAXLANDMARKS, 2]

# DB correct functions starts
def add_to_DB(*lms):      #+ return number of last landmark in DB or -1
  """ add given landmark to database
      lms - landmarks;
  """

  global DBSize
  for lm in lms:
      if DBSize+1 < MAX_LANDMARKS:   # len(landmarksDB) but it's the same
          landmarkDB[DBSize].pos[0] = lm.pos[0]    # set landmark coordinates
          landmarkDB[DBSize].pos[1] = lm.pos[1]    # set landmark coordinates
          landmarkDB[DBSize].life = LIFE           # set landmark life counter
          landmarkDB[DBSize].id_ = DBSize           # set landmark id
          landmarkDB[DBSize].total_times_observed = 1   # initialise number of times we've seen landmark
          landmarkDB[DBSize].bearing = lm.bearing  # set last bearing was seen at
          landmarkDB[DBSize].range_ = lm.range_      # set last range was seen at
          landmarkDB[DBSize].a = lm.a              # store landmarks wall equation
          landmarkDB[DBSize].b = lm.b              # store landmarks wall equation
          DBSize += 1
  return DBSize

def get_DB():           #+ return database
  """ return database with landmarks
  """

  return landmarkDB[:DBSize]

def get_DB_size():      #+ return number of landmarks in database
  """ return number of landmarks in database
  """

  return DBSize
# DB correct functions ends

def slam_id(id_):       #+ return IDtoID - array with ids (landmark_ID, slam_ID)
  """ return landmark_ID if slam_ID is exist, else -1
      id_ - landmark_ID;
  """

  for i in range(EKFLandmarks):
      if IDtoID[i][0] == id_:
          return IDtoID[i][1]

def add_slam_id(landmark_ID, slam_ID):   #+ return nothing
  """ add ids of landmark in IDtoID
  """

  global EKFLandmarks
  IDtoID[EKFLandmarks][0] = landmark_ID
  IDtoID[EKFLandmarks][1] = slam_ID
  EKFLandmarks += 1


def landmark_distance(lm1, lm2):              #+ return distance
  """ count distance between 2 landmarks
      lm1 - landmark1;
      lm2 - landmark2;
  """

  return math.sqrt((lm1.pos[0]-lm2.pos[0])**2 + (lm1.pos[1]-lm2.pos[1])**2)


def distance(x1, y1, x2, y2):                 #+ return distance
  """ count distance betweet 2 points
      x1, y1 - point 1
      x2, y2 - point 2
  """

  return math.sqrt((x1-x2)**2 + (y1-y2)**2)


def distance_to_line(x, y, a, b):             #+ return distance
  """ count distance to line
      x,y - point;
      a,b - line(y=ax+b);
  """

  if a:
    ao = -1.0 / a
  else:
    return abs(y-b)
  bo = y - ao*x
  px = (b - bo) / (ao - a)
  py = ((ao * (b-bo)) / (ao-a)) + bo
  temp = distance(x, y, px, py)

  return temp


def get_closest_association(lm):              #+ return id, total_times_observed
    """ return the closest landmark to given in DB
        lm - landmark;
    """

    closest_landmark = temp = 0
    least_distance = 99999       # 99999m is least initial distance, its big
    for landmark in get_DB():      # for i in range(DBSize)
        # only associate to landmarks we have seen more than MIN_OBSERVATIONS times
        if landmark.total_times_observed > MIN_OBSERVATIONS:
            temp = landmark_distance(lm, landmark)
            if temp < least_distance:
                least_distance = temp
                closest_landmark = landmark

    if least_distance == 99999:
        return -1, 0
    else:
        return closest_landmark.id_, closest_landmark.total_times_observed


def get_association(lm):                      # return index or -1
    """ this method needs to be improved so we use innovation as a validation gate
        currently we just check if a landmark is within some predetermined distance of a landmark in DB

        lm - landmark to associate;
    """


    for i in range(DBSize):
        if (landmark_distance(lm, landmarkDB[i])<MAX_ERROR) &
           (landmarkDB[i].id_ != -1):
            landmarkDB[i].life = LIFE
            landmarkDB[i].total_times_observed += 1
            landmarkDB[i].bearing = lm.bearing
            landmarkDB[i].range_ = lm.range_
            return landmarkDB[i].id_
    return -1


def get_line_landmark(a, b, robot_position):  #+ return landmark
    """ a,b - describe line(y=ax+b);
        robot_position - [x,y,theta]

    our goal is to calculate point on line closest to origin (0,0)
    calculate line perpendicular to input line. a*ao = -1
    """

    ao = -1.0 / a
    x = b / (ao-a)      # landmark position
    y = (ao*b) / (ao-a)
    range_ = math.sqrt(math.pow(x-robot_position[0], 2) + \
                       math.pow(y-robot_position[1], 2))
    bearing = math.atan((y-robot_position[1]) / (x-robot_position[0])) - \
              robot_position[2]
    # now do same calculation but get point on wall closest to robot instead
    # y = aox + bo => bo = y - aox
    bo = robot_position[1] - ao*robot_position[0]
    # get intersection between y = ax + b and y = aox + bo
    # so aox + bo = ax + b => aox - ax = b - bo => x = (b - bo)/(ao - a), y = ao*(b - bo)/(ao - a) + bo
    px = (b-bo) / (ao-a)
    py = ((ao * (b-bo)) / (ao-a)) + bo
    range_error = distance(robot_position[0], robot_position[1], px, py)
    bearing_error = math.atan((py - robot_position[1]) / (px - robot_position[0])) - \
                    robot_position[2]
                    # do you subtract or add robot bearing? I am not sure!
    lm = landmark()
    # convert landmark to map coordinate
    lm.pos[0] = x
    lm.pos[1] = y
    lm.range_ = range_
    lm.bearing = bearing
    lm.a = a
    lm.b = b
    lm.range_error = range_error
    lm.bearing_error = bearing_error
    # associate landmark to closest landmark.
    lm.id_, lm.total_times_observed = get_closest_association(lm)

    return lm


def least_squares_line_estimate(laserdata, robot_position, selected_points):   #+? return a,b(y=ax+b)
  """ laserdata - data from ultrasound;
      robot_position - [x, y, theta];
      selected_points - some points;

      don't know, what this function return
  """

  # print(selected_points)
  array_size = len(selected_points)
  y = x = sumY = sumYY = sumX = sumXX = sumYX = 0
  for i in range(array_size):
    x = (math.cos((selected_points[i]*DEGREES_PER_SCAN*CONV) + (robot_position[2]*CONV)) *
        laserdata[int(selected_points[i])]) + robot_position[0]
    y = (math.sin((selected_points[i]*DEGREES_PER_SCAN*CONV) + (robot_position[2]*CONV)) *
        laserdata[int(selected_points[i])]) + robot_position[1]
    sumY += y
    sumX += x
    sumYY += y*y
    sumXX += x*x
    sumYX += y*x
  try:
    b = (sumY*sumXX - sumX*sumYX) / (array_size*sumXX - sumX*sumX)      # дисперсия
  except ZeroDivisionError:
    b = 0
  try:
    a = (array_size*sumYX - sumX*sumY) / (array_size*sumXX - sumX*sumX)   # погрешности
  except ZeroDivisionError:
    a = 0

  return a, b


def ransac_algorithm(laserdata, robot_position):       # return found_landmarks
  """ laserdata - data from ultrasound, [2.0, 2.1, 2.1, 2.1, 2.2]
      robot_position - [x,y,theta]

      main algorithm, correct robot_position with laserdata
  """

  la = [0 for i in range(100)]
  lb = [0 for i in range(100)]
  total_lines = 0
  laser_len = len(laserdata)
  # array of laser data points corresponding to the seen lines
  linepoints = [i for i in range(laser_len)]
  total_linepoints = laser_len
  # have a large array to keep track of found landmarks
  temp_landmarks = [landmark() for x in range(400)]

  # RANSAC algorithm starts
  no_trials = 0
  while ((no_trials < MAX_TRIALS) & (total_linepoints > MIN_LINEPOINTS)):

    # 1.1 choose MAX_SAMPLE points
    rnd_selected_points = [0 for i in range(MAX_SAMPLE)]
    temp = 0
    newpoint = False
    # choose one random point
    center_point = random.randint(MAX_SAMPLE, total_linepoints-1)
    rnd_selected_points[0] = center_point
    for i in range(MAX_SAMPLE):
      temp = center_point + random.random() * random.randint(0, MAX_SAMPLE)
      for j in range(i):
        if (rnd_selected_points[j] == temp):
          break    # point has already been selected
      rnd_selected_points[i] = temp

    # 1.2 compute model M1
    a = b = 0
    a, b = least_squares_line_estimate(laserdata, robot_position, rnd_selected_points)
    consensus_points = [0 for i in range(laser_len)]    # points which are close to line
    total_consensus_points = 0
    new_linepoints = [0 for i in range(laser_len)]      # points which aren't close to line
    total_new_linepoints = 0
    x = y = d = 0

    for i in range(total_linepoints):
      # convert ranges and bearing to coordinates
      x = (math.cos((linepoints[i]*DEGREES_PER_SCAN*CONV) + robot_position[2]*CONV) *
          laserdata[linepoints[i]]) + robot_position[0]
      y = (math.sin((linepoints[i]*DEGREES_PER_SCAN*CONV) + robot_position[2]*CONV) *
          laserdata[linepoints[i]]) + robot_position[1]
      d = distance_to_line(x, y, a, b)
      if (d < RANSAC_TOLERANCE):
        # add points which are close to line
        consensus_points[total_consensus_points] = linepoints[i]
        total_consensus_points += 1
      else:
        # add points which are not close to line
        new_linepoints[total_new_linepoints] = linepoints[i]
        total_new_linepoints += 1

    if total_consensus_points > RANSAC_CONSENSUS:
        a, b = least_squares_line_estimate(laserdata, robot_position,
                                       consensus_points)
        for i in range(total_consensus_points):
            linepoints = new_linepoints # new_linepoints.CopyTo(linepoints, 0)
            total_linepoints = total_new_linepoints
        la[total_lines] = a
        lb[total_lines] = b
        total_lines += 1
        no_trials = 0
    else:
        no_trials += 1

  for i in range(total_lines):
      #print(type(get_line_landmark(la[i], lb[i], robot_position)), type(temp_landmarks[i]))
      temp_landmarks[i] = get_line_landmark(la[i], lb[i], robot_position)

  found_landmarks = [landmark() for i in range(total_lines)]

  for i in range(len(found_landmarks)):
     found_landmarks[i] = temp_landmarks[i]
     # foundLandmarks[i] = (landmark)tempLandmarks[i]

  return found_landmarks


def remove_bad_landmarks(laserdata, robot_position):   # return 0
  maxrange = 0
  for i in range(len(laserdata)-1):
  # distance further away than 8.1m we assume are failed returns
  # we get the laser data with max range
    if laserdata[i-1]<8.1 & laserdata[i+1]<8.1 & laserdata[i]>maxrange:
        maxrange = laserdata[i]

  # maxrange = MAX_RANGE
  xbounds = [0, 0, 0, 0]
  ybounds = [0, 0, 0, 0]
  # get bounds of rectangular box to remove bad landmarks from
  xbounds[0] = math.cos((1*DEGREES_PER_SCAN*CONV) + (robot_position[2]*CONV)) * \
               maxrange + robot_position[0]
  ybounds[0] = math.sin((1*DEGREES_PER_SCAN*CONV) + (robot_position[2]*CONV)) * \
               maxrange + robot_position[1]
  xbounds[1] = xbounds[0] + math.cos((180*DEGREES_PER_SCAN*CONV) + \
                                     (robot_position[2]*CONV)) * maxrange
  ybounds[1] = ybounds[0] + math.sin((180*DEGREES_PER_SCAN*CONV) + \
                                     (robot_position[2]*CONV)) * maxrange
  xbounds[2] = math.cos((359*DEGREES_PER_SCAN*CONV) + \
                        (robot_position[2]*CONV)) * maxrange + robot_position[0]
  ybounds[2] = math.sin((359*DEGREES_PER_SCAN*CONV) + \
                        (robot_position[2]*CONV)) * maxrange + robot_position[1]
  xbounds[3] = xbounds[2] + math.cos((180*DEGREES_PER_SCAN*CONV) + \
                                     (robot_position[2]*CONV)) * maxrange
  ybounds[3] = ybounds[2] + math.sin((180*DEGREES_PER_SCAN*CONV) + \
                                     (robot_position[2]*CONV)) * maxrange
  # now check DB for landmarks that are within this box
  # decrease life of all landmarks in box.
  # If the life reaches zero, remove landmark
  pntx = pnty = 0
  for k in range(DBSize+1):
    pntx = landmarkDB[k].pos[0]
    pnty = landmarkDB[k].pos[1]
    i = j = 0
    if (robot_position[0]<0 | robot_position[1]<0): in_rectangle = False
    else: in_rectangle = True

    for i in range(4):
        if ( (((ybounds[i] <= pnty) & (pnty < ybounds[j])) |
              ((ybounds[j] <= pnty) & (pnty < ybounds[i]))) &
             (pntx < ((xbounds[j] - xbounds[i]) *
                      (pnty - ybounds[i]) /
                      (ybounds[j] - ybounds[i]) +
                      xbounds[i]
                     )
             )
           ):
            in_rectangle = not(in_rectangle)
        j = i
        i += 1

    if(inRectangle):     # in rectangle so decrease life and maybe remove
        landmarkDB[k].life -= 1
        if landmarkDB[k].life <= 0:
            del(landmark[k])
            DBSize -= 1

  return 0
