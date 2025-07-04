import os

def modify_txt_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    modified_lines = []
    for line in lines:
        parts = line.split()
        if parts and parts[0] != '0':
            parts[0] = '0'
        modified_lines.append(' '.join(parts) + '\n')

    with open(file_path, 'w') as file:
        file.writelines(modified_lines)

def process_directory(directory):
    for filename in os.listdir(directory):
        if filename.endswith('.txt'):
            file_path = os.path.join(directory, filename)
            modify_txt_file(file_path)

if __name__ == "__main__":
    directory_path = 'E:/Dataset/obstacle/labels/train'
    process_directory(directory_path)
    print("All txt files have been processed.")
