set size ratio -1
plot \
    "dump/amcl.txt" using 1:2 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "AMCL", \
    "dump/navsatfix_measurements.txt" using 1:2 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "GNSS"
pause 0.2
reread