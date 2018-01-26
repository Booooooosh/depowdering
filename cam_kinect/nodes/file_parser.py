# This script parse the data file and
# return a dictionary that can be sent
# to the blackbox

import sys

def parse_line(line, container):
  point = line.split(',')
  container.append(float(point[0]))
  container.append(float(point[1]))
  container.append(float(point[2]))
  return

def file_parser(path_to_file):
  data_dict = {'rows': 0, 'cols': 0, 'data': []}
  cols = 0
  rows = 1
  data = []
  data_per_row = []
  # Try openning the file
  try:
    with open(path_to_file, 'r') as data_file:
      raw_data = data_file.read().splitlines()
      for pointstr in raw_data:
        # See if we are getting a new row
        if pointstr.find('|') != -1:
          data.append(data_per_row)
          data_per_row = []
          rows += 1
          cols = 0
        # Still getting points for the same row
        pointstr = pointstr[pointstr.find('(') + 1 : pointstr.find(')')]
        parse_line(pointstr, data_per_row)
        cols += 1
      # Add the final row into the data
      data.append(data_per_row)
      data_dict['data'] = data
      data_dict['rows'] = rows
      data_dict['cols'] = cols
  except IOError as io_err:
    print 'File not found. PATH = %s' % path_to_file
    return None
  else:
    print '[File Parser] Success.'
    return data_dict

if __name__ == '__main__':
  data_dict = file_parser(sys.argv[1])
  print data_dict
