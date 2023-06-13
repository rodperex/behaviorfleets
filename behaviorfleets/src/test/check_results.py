import os
import math

path = '/results'
path = os.getcwd() + path

bbs = []

for file_path in os.listdir(path): 
  with open(path + '/' + file_path) as f:
    if (not('_handler' in file_path) and not('waiting_times' in file_path)):
      lines = f.readlines()
      d = {}
      for line in lines:
        pairs = (line.rstrip('\n')).split(':')
        d[pairs[0]] = pairs[1]
      if file_path == 'manager.txt':
        manager = d
      else:
        bbs.append(d)

all_same = all(manager == item for item in bbs)
print(all_same)

wts = []

for file_path in os.listdir(path):
  with open(path + '/' + file_path) as f:
    if 'handler' in file_path:
      lines = f.readlines()
      d = {}
      for line in lines:
        pairs = (line.rstrip('\n')).split(' = ')
        wts.append(pairs[1])

wts = [float(x) for x in wts]
wts = [x for x in wts if not math.isnan(x)]
wts = [x / 1e3 for x in wts]  

print('CLIENT SIDE')
print('avg', end=" = ")
print(sum(wts)/len(wts))
print('max', end=" = ")
print(max(wts))
print('min', end=" = ")
print(min(wts))


path = path + '/waiting_times.txt'
wts = []

with open(path) as f:
  lines = f.readlines()
  for line in lines:
    line = line.rstrip('\n')
    wts.append(float(line))

wts = [float(x) for x in wts]
wts = [x for x in wts if not math.isnan(x)]
wts = [x / 1e3 for x in wts] 

print('SERVER SIDE')
print('avg', end=" = ")
print(sum(wts)/len(wts))
print('max', end=" = ")
print(max(wts))
print('min', end=" = ")
print(min(wts))

