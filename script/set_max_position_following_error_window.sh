ethercat download -p8  0x6065 0 520000   #left arm J1
ethercat download -p9  0x6065 0 520000   #left arm J1
ethercat download -p10 0x6065 0 520000   #left arm J2
ethercat download -p11 0x6065 0 520000   #left arm J3
ethercat download -p12 0x6065 0 520000  #left arm J4 (elbow)
ethercat download -p13 0x6065 0 520000   #left arm J5
ethercat download -p14 0x6065 0 520000  #left arm J6
ethercat download -p15 0x6065 0 520000  #left arm J7
ethercat download -p16 0x6065 0 520000   #right arm J1
ethercat download -p17 0x6065 0 520000   #right arm J2
ethercat download -p18 0x6065 0 520000   #right arm J3
ethercat download -p19 0x6065 0 520000   #right arm J4 (elbow)
ethercat download -p20 0x6065 0 520000   #right arm J5
ethercat download -p21 0x6065 0 520000   #right arm J6

ethercat download -p8  0x3b60 0 520000   #left arm J1
ethercat download -p9  0x3b60 0 520000   #left arm J1
ethercat download -p10 0x3b60 0 520000   #left arm J2
ethercat download -p11 0x3b60 0 520000   #left arm J3
ethercat download -p12 0x3b60 0 520000  #left arm J4 (elbow)
ethercat download -p13 0x3b60 0 520000   #left arm J5
ethercat download -p14 0x3b60 0 520000  #left arm J6
ethercat download -p15 0x3b60 0 520000  #left arm J7
ethercat download -p16 0x3b60 0 520000   #right arm J1
ethercat download -p17 0x3b60 0 520000   #right arm J2
ethercat download -p18 0x3b60 0 520000   #right arm J3
ethercat download -p19 0x3b60 0 520000   #right arm J4 (elbow)
ethercat download -p20 0x3b60 0 520000   #right arm J5
ethercat download -p21 0x3b60 0 520000   #right arm J6

echo '0x6065'
echo 8
ethercat upload -p8  0x6065 0   #left arm J1
echo 9
ethercat upload -p9  0x6065 0   #left arm J1
echo 10
ethercat upload -p10 0x6065 0   #left arm J2
echo 11
ethercat upload -p11 0x6065 0   #left arm J3
echo 12
ethercat upload -p12 0x6065 0  #left arm J4 (elbow)
echo 13
ethercat upload -p13 0x6065 0   #left arm J5
echo 14
ethercat upload -p14 0x6065 0  #left arm J6
echo 15
ethercat upload -p15 0x6065 0  #left arm J7
echo 16
ethercat upload -p16 0x6065 0   #right arm J1
echo 17
ethercat upload -p17 0x6065 0   #right arm J2
echo 18
ethercat upload -p18 0x6065 0   #right arm J3
echo 19
ethercat upload -p19 0x6065 0   #right arm J4 (elbow)
echo 20
ethercat upload -p20 0x6065 0   #right arm J5
echo 21
ethercat upload -p21 0x6065 0   #right arm J6

echo
echo '0x3b60'
echo 8
ethercat upload -p8  0x3b60 0   #left arm J1
echo 9
ethercat upload -p9  0x3b60 0   #left arm J1
echo 10
ethercat upload -p10 0x3b60 0   #left arm J2
echo 11
ethercat upload -p11 0x3b60 0   #left arm J3
echo 12
ethercat upload -p12 0x3b60 0  #left arm J4 (elbow)
echo 13
ethercat upload -p13 0x3b60 0   #left arm J5
echo 14
ethercat upload -p14 0x3b60 0  #left arm J6
echo 15
ethercat upload -p15 0x3b60 0  #left arm J7
echo 16
ethercat upload -p16 0x3b60 0   #right arm J1
echo 17
ethercat upload -p17 0x3b60 0   #right arm J2
echo 18
ethercat upload -p18 0x3b60 0   #right arm J3
echo 19
ethercat upload -p19 0x3b60 0   #right arm J4 (elbow)
echo 20
ethercat upload -p20 0x3b60 0   #right arm J5
echo 21
ethercat upload -p21 0x3b60 0   #right arm J6