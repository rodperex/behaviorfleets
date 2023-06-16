import os
import math

path = '/results'
path = os.getcwd() + path

bbs = []
n_changes = []

for file_path in os.listdir(path): 
  with open(path + '/' + file_path) as f:
    if ('~' in f.name or 'xlsx' in f.name):
        continue
    if (not('_handler' in file_path) and not('waiting_times' in file_path)):
      lines = f.readlines()
      d = {}
      for line in lines:
        pairs = (line.rstrip('\n')).split(':')
        if pairs[0] == "n_changes":
          n_changes.append(int(pairs[1]))
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
        else:
          succ.append(pairs[1])
          i = 0


succ = [int(x) for x in succ]
wts = [float(x) for x in wts]
wts = [x for x in wts if not math.isnan(x)]
wts = [x / 1e3 for x in wts]  

print('CLIENT SIDE')
print('avg wt', end=" = ")
print(sum(wts)/len(wts))
print('max wt', end=" = ")
print(max(wts))
print('min wt', end=" = ")
print(min(wts))
print('avg succ', end=" = ")
print(sum(succ)/len(succ))
print('max succ', end=" = ")
print(max(succ))
print('min succ', end=" = ")
print(min(succ))
print('avg changes', end=" = ")
print(sum(n_changes)/len(n_changes))  
print('max changes', end=" = ")
print(max(n_changes))
print('min changes', end=" = ")
print(min(n_changes))



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

print('SERVER SIDE')
print('avg wt', end=" = ")
print(sum(wts)/len(wts))
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
print(sum(wts)/len(wts))
print(max(wts))
print(min(wts))
print(max_q)
print(sum(succ)/len(succ))
print(max(succ))
print(min(succ))
print(sum(n_changes)/len(n_changes))  
print(max(n_changes))
print(min(n_changes))
