import sys
import subprocess

apriltag_fn = '../matlab/aprilTag/getAprilTag'

# Log file name
fname=sys.argv[1];
new_log_fname = sys.argv[2];


# Open it
log_file = open(fname);
new_log = open(new_log_fname,'w');
lines = log_file.readlines();

for i in lines:
	line = i.strip('\n').split(' ');	
	if  line[2] !=  '0':
		in_file = line[2];
	        out_file = in_file.split('.');
		out_file = out_file[0]+'.txt';
		out_line = line[0] + ' ' + line[1] + ' ' + out_file + '\n';
		new_log.write(out_line);
                command = [apriltag_fn, in_file, out_file];
                subprocess.call(command);

	else:
		new_log.write(i);
		


		





