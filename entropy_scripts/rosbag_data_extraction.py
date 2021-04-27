import argparse
import sys
import time
import os
import matplotlib.pyplot as plt
import rosbag_pandas
import pandas as pd


def parser_function():
    parser = argparse.ArgumentParser(
        description='Transform rosbags to folders with csv topic files.')

    parser.add_argument(
        "-p", "--path", help="This is the path to the bagfiles to transform", type=str, required=True)

    parser.add_argument(
        "-b", "--bag", help="Single bag file to transform", type=str)

    parser.add_argument("-a", "--all_bags", help="Transform all the bags in the folder.",
                        action="store_true")

    parser.add_argument("-t", "--topics",
                        help="Select topics to transform to csv files. If not specified all topics are going to be transformed.",
                        nargs='*')

    args = parser.parse_args()

    if not (args.bag or args.all_bags):
        parser.error(
            'Please insert argument for a single rosbag (-b, --bag) or all rosbags (-a, --all_bags')
        sys.exit()

    print('The path is:', args.path)
    print('The bag is:', args.bag)
    print('The all_bags argument is:', args.all_bags)
    print('The topics are:', args.topics)

    return args.path, args.bag, args.all_bags, args.topics, args.print_topic


if __name__ == "__main__":

    complete_df = pd.DataFrame()
    path, rosbag_file, all_bags, topic_list, print_topic = parser_function()

    if not all_bags:
        listOfBagFiles = [rosbag_file]
        numberOfFiles = "1"
        print("\nReading only 1 bagfile: " + str(listOfBagFiles))

    elif all_bags:
        # get list of only bag files in current dir.
        listOfBagFiles = [f for f in os.listdir(path) if f[-4:] == ".bag"]
        numberOfFiles = str(len(listOfBagFiles))
        print("\nReading all " + numberOfFiles +
              " bagfiles in current directory: \n")
        for f in listOfBagFiles:
            print(f)

    # # Print
    # print("\nPress ctrl+c in the next 10 seconds to cancel \n")
    # # time.sleep(10)

    # fig, ax = plt.subplots(len(listOfBagFiles), 1)
    # fig.suptitle(
    #     'Three teleop experiments. Topic <teleop/cmd_vel/angular.z>')

    count = 0
    for bagFile in listOfBagFiles:
        count += 1
        print("reading file " + str(count) +
              " of  " + numberOfFiles + ": " + bagFile)

        # create a new directory
        folder = path+bagFile.split(".")[0]+"/"
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass

        # access bag
        if topic_list:
            df = rosbag_pandas.bag_to_dataframe(
                path+bagFile, include=topic_list)
        else:
            df = rosbag_pandas.bag_to_dataframe(
                path+bagFile)

        # # Print
        # ax[count-1].title.set_text(bagFile)
        # ax[count-1].plot(df["/teleop/cmd_vel/angular/z"])

        filename = folder + bagFile.split(".")[0] + '.csv'
        df.to_csv(filename, index=True)

    print "Done reading all " + numberOfFiles + " bag files."

    plt.show()
