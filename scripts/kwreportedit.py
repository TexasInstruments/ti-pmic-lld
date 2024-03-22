import os

def getPathToDriver() -> str:
    return os.path.dirname(os.path.dirname(os.path.dirname(__file__)))

if __name__ == "__main__":
    kwReport = "kw_report.xml"
    pathToDriver = getPathToDriver()
    
    # As we are about to load the report in memory, 
    # ensure that the report is not over 10 MB
    if (os.path.getsize(kwReport) > 10000000):
        print(__file__ + " error: file size greater than 10 MB")
        exit(-1)

    # Store the KW report in memory
    with open(kwReport, 'r') as file:
        data = file.readlines()

    # Change the KW report encoding to UTF-8
    data[0] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"

    # Delete any occurrence of the path to the driver 
    for i in range(1, len(data)):
        data[i] = data[i].replace(pathToDriver, "")

    # Write the new version of the report to disk
    with open(kwReport, 'w') as file:
        file.writelines(data)
