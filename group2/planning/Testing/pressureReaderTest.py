import time
import cPickle as pickle

if __name__ == "__main__":
    while True:
        with open("pressureReading.pkl", "rb") as f:
            val = pickle.load(f)
        print val
        time.sleep(1)
