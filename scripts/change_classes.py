import os
# os.path.dirname(r"D:\Darknet\underwater_training_data")

directory = r"D:\Darknet\mclab_badge_and_spy_close_wiggle_labeled"
""" 
for path in os.listdir(directory):
    if path.endswith(".txt"):
        if path.__eq__("classes.txt"):
            pass
        else:
            print(path)

            replacement = ""
            with open(directory + "/" + path, "r") as file:
                for line in file.readlines():
                    if line == "\n":
                        pass
                    else:
                        change = line.strip()
                        replacement = replacement + change + "\n"

                file.close()

            with open(directory + "/" + path, "w") as file:
                file.write(replacement)
                file.close()
 """
counter = 0

for path in os.listdir(directory):
    if path.endswith(".txt"):
        counter += 1

        if counter == 150:
            lines = []
            with open(directory + "/" + path, "r") as file:
                # print(file.readlines())
                lines = file.readlines()
                print(lines)

                for line in lines:
                    print(line[0])

                break
