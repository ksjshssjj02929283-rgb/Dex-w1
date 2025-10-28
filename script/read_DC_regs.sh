watch -n0.1 "\
  echo 'ANKLE 1';\
  ethercat reg_read -p 1 0x092c -t int32; \
  echo 'KNEE 2';\
  ethercat reg_read -p 2 0x092c -t int32; \
  echo 'BUTTOCK 3';\
  ethercat reg_read -p 3 0x092c -t int32; \
  echo 'WAIST 4';\
  ethercat reg_read -p 4 0x092c -t int32; \
  echo 'NECK1 7';\
  ethercat reg_read -p 7 0x092c -t int32; \
  echo 'NECK2 8';\
  ethercat reg_read -p 8 0x092c -t int32; \
  echo 'LEFT J1 9';\
  ethercat reg_read -p 9 0x092c -t int32; \
  echo 'LEFT J2 10';\
  ethercat reg_read -p 10 0x092c -t int32; \
  echo 'LEFT J3 11';\
  ethercat reg_read -p 11 0x092c -t int32; \
  echo 'LEFT J4 12';\
  ethercat reg_read -p 12 0x092c -t int32; \
  echo 'LEFT J5 13';\
  ethercat reg_read -p 13 0x092c -t int32; \
  echo 'LEFT J6 14';\
  ethercat reg_read -p 14 0x092c -t int32; \
  echo 'LEFT J7 15';\
  ethercat reg_read -p 15 0x092c -t int32; \
  echo 'RIGHT J1 16';\
  ethercat reg_read -p 16 0x092c -t int32; \
  echo 'RIGHT J2 17';\
  ethercat reg_read -p 17 0x092c -t int32; \
  echo 'RIGHT J3 18';\
  ethercat reg_read -p 18 0x092c -t int32; \
  echo 'RIGHT J4 19';\
  ethercat reg_read -p 19 0x092c -t int32; \
  echo 'RIGHT J5 20';\
  ethercat reg_read -p 20 0x092c -t int32; \
  echo 'RIGHT J6 21';\
  ethercat reg_read -p 21 0x092c -t int32; \
  echo 'RIGHT J7 22';\
  ethercat reg_read -p 22 0x092c -t int32; \
  "