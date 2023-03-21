import yaml

def update_gains(kp,kd,ki,k=0,filepath = ""):
    with open(filepath,"w") as f:
        try:
            gains = {'Kp':kp,'Kd':kd,'Ki':ki,'K':k}
            yaml.dump(gains,f)
            f.close()

        except yaml.YAMLError as exc:
            print(exc)
            print(f'Failed to load PID terms from {filepath}')


def load_gains(filepath)  -> dict:
    with open(filepath,"r") as f:
        gains = yaml.full_load(f)
        f.close()
    return gains