set size ratio -1
plot \
    "dump/amcl.txt" using 2:1 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "AMCL", \
    "dump/converted_gnss.txt" using 2:1 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "Converted GNSS"
pause 0.2
reread