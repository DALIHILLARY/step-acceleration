import matplotlib.pyplot as plt
import os

# Get the current working directory
cwd = os.getcwd()

# Iterate through all files in the current directory
for filename in os.listdir(cwd):
    # Check if the filename ends with "pos.txt"
    if filename.endswith("pos-sun.txt"):
        # Do something with the file
        print(f"Processing file {filename}")

        # Open the text file
        with open(filename, 'r') as file:
            # Read all lines from the file
            lines = file.readlines()


        # Create a list of x values incrementing by 1 for each line
        x_values = [float(line.strip().split(",")[0]) for line in lines]

        # Create a list of y values by stripping the newline character from each line
        y_values = [float(line.strip().split(",")[1]) for line in lines]

        # Plot the y values against the x values
        plt.plot(x_values, y_values)

        plt.title(filename.replace(".txt", ""))
        plt.ylabel("Relative Position (m)")
        plt.xlabel("Relative Postion (m)")

        plt.savefig(filename.replace(".txt", ".png"))


        # Show the plot
        plt.show()
