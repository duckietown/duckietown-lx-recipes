import yaml

def update_gains(kp,kd,ki,k=0,filepath = ""):
    with open(filepath,"w") as f:
        gains = {'Kp':kp,'Kd':kd,'Ki':ki,'K':k}
        yaml.dump(gains,f)
        f.close()

def load_gains(filepath)  -> dict:
    with open(filepath,"r") as f:
        gains = yaml.full_load(f)
        f.close()
    return gains