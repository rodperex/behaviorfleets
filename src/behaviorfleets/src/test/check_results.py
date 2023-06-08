import os

path = '/src/behaviorfleets/results/'
path = os.getcwd() + path

bbs = []

for file_path in os.listdir(path):
  print(file_path)
  
  with open(path + '/' + file_path) as f:
    lines = f.readlines()
    d = {}
    for line in lines:
      pairs = (line.rstrip('\n')).split(':')
      d[pairs[0]] = pairs[1]
    if file_path == 'manager.txt':
      manager = d
    else:
      bbs.append(d)

print(manager)
all_same = all(manager == item for item in bbs)
print(all_same)
