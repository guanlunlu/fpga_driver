$Dir = Get-Location
echo $Dir
scp -r $Dir/src $Dir\include $Dir\CMakeLists.txt $Dir\config $Dir\fpga_bitfile admin@169.254.105.68:~/corgi_ws/src/fpga_server