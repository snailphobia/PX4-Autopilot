import re
import json
import matplotlib.pyplot as plt
import time

plt.ion()  # Turn on interactive mode

x = []  # List to hold x-values (optional)
y = []  # List to hold y-values

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('patternname', help='the name of the file to read')
args = parser.parse_args()

while True:
    with open('gdb.txt', 'r') as file:
        content = file.read()
    print("file2")
    pattern = r'~"\[DEBUGLOG '+ args.patternname + '\]\((.*?)\)"'
    matches = re.findall(pattern, content)

    data = [eval(match) for match in matches]
    data = [d for d in data]  # Add new data to y-values
    if(len(data) > 100):
        y = data[-100:]
    x = list(range(len(data)-len(y),len(data)))  # Update x-values to match length of y-values

    plt.clf()  # Clear the current figure
    plt.plot(x, y)  # Plot the data
    plt.draw()  # Draw the plot
    plt.pause(0.1)  # Pause for a short period

    time.sleep(1)  # Wait before reading the file again
