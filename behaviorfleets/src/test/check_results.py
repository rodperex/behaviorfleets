import os
import math
import numpy as np

path = '/results'
path = os.getcwd() + path

bbs = []
n_updates = []

for file_path in os.listdir(path): 
  with open(path + '/' + file_path) as f:
    if ('~' in f.name or 'xlsx' in f.name):
        continue
    if (not('_handler' in file_path) and not('waiting_times' in file_path) and not('experiments' in file_path)):
      lines = f.readlines()
      d = {}
      for line in lines:
        pairs = (line.rstrip('\n')).split(':')
        if pairs[0] == "n_changes":
          n_updates.append(int(pairs[1]))
        else:
          d[pairs[0]] = pairs[1]
      if file_path == 'manager.txt':
        manager = d
      else:
        bbs.append(d)

all_same = all(manager == item for item in bbs)
print(all_same)

wts = []
succ = []
reqs = []

for file_path in os.listdir(path):
  with open(path + '/' + file_path) as f:
    if 'handler' in file_path:
      lines = f.readlines()
      d = {}
      i = 0
      for line in lines:
        pairs = (line.rstrip('\n')).split(':')
        if i == 0:
          wts.append(pairs[1])
          i += 1
        elif i == 1:
          reqs.append(pairs[1])
          i += 1
        else:
          succ.append(pairs[1])
          i = 0
  

succ = [int(x) for x in succ]
reqs = [int(x) for x in reqs]
wts = [float(x) for x in wts]
wts = [x for x in wts if not math.isnan(x)]
wts = [x / 1e3 for x in wts]  

print('-' * 10)
print('-' * 10)
print('CLIENT SIDE')
print('-' * 10)
print('avg wt', end=" = ")
print(np.mean(wts))
print('std wt', end=" = ")
print(np.std(wts))
print('max wt', end=" = ")
print(max(wts))
print('min wt', end=" = ")
print(min(wts))
print('-' * 10)
print('avg reqs', end=" = ")
print(np.mean(reqs))
print('std reqs', end=" = ")
print(np.std(reqs))
print('max reqs', end=" = ")
print(max(reqs))
print('min reqs', end=" = ")
print(min(reqs))
print('-' * 10)
print('avg succ', end=" = ")
print(np.mean(succ))
print('std succ', end=" = ")
print(np.std(succ))
print('max succ', end=" = ")
print(max(succ))
print('min succ', end=" = ")
print(min(succ))
print('-' * 10)
print('avg updates', end=" = ")
print(np.mean(n_updates))
print('std updates', end=" = ")
print(np.std(n_updates))
print('max updates', end=" = ")
print(max(n_updates))
print('min updates', end=" = ")
print(min(n_updates))


path = path + '/waiting_times.txt'
wts = []

with open(path) as f:
  lines = f.readlines()
  for line in lines:
    line = line.rstrip('\n')
    wts.append(float(line))

max_q = int(wts[-1])
wts.pop()
wts = [float(x) for x in wts]
wts = [x for x in wts if not math.isnan(x)]
wts = [x / 1e3 for x in wts] 

print('-' * 10)
print('-' * 10)
print('SERVER SIDE')
print('-' * 10)
print('avg wt', end=" = ")
print(np.mean(wts))
print('std wt', end=" = ")
print(np.std(wts))
print('max wt', end=" = ")
print(max(wts))
print('min wt', end=" = ")
print(min(wts))
print('max_q', end=" = ")
print(max_q)

# to paste it in excel
print("\n\n------------------")
print('TO EXCEL', end="\n------------------\n")
print(all_same)
print(np.mean(wts))
print(np.std(wts))
print(max(wts))
print(min(wts))
print(np.mean(succ))
print(np.std(succ))
print(max(succ))
print(min(succ))
# print(np.mean(succ) / np.mean(reqs))
psucc_req = [i / j for i, j in zip(succ, reqs)]
print(np.mean(psucc_req))
# print(np.mean(succ) / np.mean(n_updates))
print(np.mean(n_updates))
print(np.std(n_updates)) 
print(max(n_updates))
print(min(n_updates))
print(max_q)
