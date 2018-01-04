export LD_LIBRARY_PATH=./ 
./mjpg_streamer -i "input_file.so -f /mnt/RAMDisk/ -n thermal.jpg" -o "output_http.so -p 8080 -w /usr/local/www" &
./mjpg_streamer -i "input_file.so -d 250 -f /mnt/RAMDisk/ -n real.jpg"    -o "output_http.so -p 8081 -w /usr/local/www"
