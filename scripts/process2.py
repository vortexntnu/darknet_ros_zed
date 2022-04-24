import glob
import os  # Current directory

# Percentage of images to be used for the test set
train_dir = r"D:\Darknet\mclab_badge_and_spy_close_wiggle_labeled"
test_dir = r"D:\Darknet\mclab_badge_and_spy_training_data_labeled"

# Populate train.txt and test.txt
file_train = open('train_mclab_badge_and_spy2.txt', 'w')
file_test = open('test_mclab_badge_and_spy2.txt', 'w')

for pathAndFilename in glob.iglob(os.path.join(train_dir, "*.jpg")):
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))
    file_train.write(train_dir + "/" + title + '.jpg' + "\n")


for pathAndFilename in glob.iglob(os.path.join(test_dir, "*.jpg")):
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))
    file_test.write(test_dir + "/" + title + '.jpg' + "\n")
