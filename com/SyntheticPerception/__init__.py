
import importlib
import sys
print ("[CUSTOM] Reloading...")
L = list(sys.modules.keys())
for k in L:
  if "com.copycat" in k:
    print (k)
    importlib.reload(sys.modules[k])
