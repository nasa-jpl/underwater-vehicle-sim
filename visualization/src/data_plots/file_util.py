def load_csv(filename):
    data = {}
    header = []
    with open(filename) as f:
        first = True
        for line in f:
            if first:
                header = line.split(",")
                header = [h.strip() for h in header]
                for h in header:
                    data[h] = []
                first = False
            else:
                splitLine = line.split(",")
                for i, h in enumerate(splitLine):
                    try:
                         data[header[i]].append(float(h.strip()))
                    except ValueError:
                         data[header[i]].append(h.strip())
    return data

def load_csv_first_and_last(filename):
    data = {}
    header = []
    with open(filename) as f:
        headerLine = f.readline()
        header = headerLine.split(",")
        header = [h.strip() for h in header]
        for h in header:
            data[h] = []

        firstLine = f.readline()
        firstLine = firstLine.split(",")
        for i, h in enumerate(firstLine):
            try:
                 data[header[i]].append(float(h.strip()))
            except ValueError:
                 data[header[i]].append(h.strip())
                
        for line in f:
            pass
        lastLine = line
        lastLine = lastLine.split(",")
        for i, h in enumerate(lastLine):
            try:
                 data[header[i]].append(float(h.strip()))
            except ValueError:
                 data[header[i]].append(h.strip())

    return data



def load(filename):
    if filename.endswith(".csv"):
        return load_csv(filename)

    return None