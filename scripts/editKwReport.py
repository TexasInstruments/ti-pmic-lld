import os

if __name__ == "__main__":
    kwReport = "kw_report.xml"
    pathToDriver = "C:\\Users\\a0500030\\workspace_v12\\burton_rtos"
    
    # Check if KW report exists
    if (not os.path.exists(kwReport)):
        print(__file__ + " error: kw_report.xml does not exist")
        exit(-1)

    # Check if path to driver exists
    if (not os.path.exists(pathToDriver)):
        print(__file__ + " error: path to driver does not exist")
        exit(-2)

    # As we are about to load the report in memory, 
    # ensure that the report is not over 10 MB
    if (os.path.getsize(kwReport) > 10000000):
        print(__file__ + " error: file size greater than 10 MB")
        exit(-3)

    # Overwrite first line of KW report to change the encoding;
    # Delete any occurances of the path to pmic_drv
    with open(kwReport, 'r') as file:
        data = file.readlines()
    data[0] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    for i in range (1, len(data)):
        data[i] = data[i].replace(pathToDriver, "")
    with open(kwReport, 'w') as file:
        file.writelines(data)
