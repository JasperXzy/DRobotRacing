from os import makedirs


DATASET_NMAE = "track_dataset"


path = "./" + DATASET_NMAE + "/"
try:
    makedirs(path + "train/image/")
    makedirs(path + "train/label/")
    makedirs(path + "test/image/")
    makedirs(path + "test/label/")
    print("Dirs Success")
except:
    print("Dirs Failed")
