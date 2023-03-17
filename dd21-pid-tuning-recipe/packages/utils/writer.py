import yaml

def update_gains(kp,kd,ki,filepath):
    with open(filepath,"w") as f:
        gains = {'kp':kp,'kd':kd,'ki':ki}
        yaml.dump(gains,f)
        f.close()

def load_gains(filepath):
    with open(filepath,"r") as f:
        gains = yaml.full_load(f)
        f.close()
    return gains['kp'], gains['kd'], gains['ki']