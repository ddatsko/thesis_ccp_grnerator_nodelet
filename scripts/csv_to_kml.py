import sys
import os


def main():
    if len(sys.argv) < 3:
        print("Error. Usage:$ python csv_to_kml.py <input_csv_file> <output_kml_file>")
        sys.exit(-1)

    if not os.path.exists('template.kml'):
        print("Error: cannot find 'template.kml' file for using as a template")
        sys.exit(-1)
    template = open('template.kml', 'r').read()
    points_str = ' '.join(list(map(lambda x: ''.join(list(filter(lambda c: c != ' ', x.strip()))),open(sys.argv[1], 'r').readlines())))
    template = template.replace("%%%%", points_str)
    with open(sys.argv[2], 'w') as f:
        f.write(template)
    



if __name__ == '__main__':
    main()
