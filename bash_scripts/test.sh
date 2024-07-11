count=$(<"/tmp/t_counter")
echo $count

# Increment count safely using arithmetic expansion
count=`expr $count + 1`
#echo $count
# Check if count is not equal to 1 (using double equals)
if [[ $count -ne 1 ]]; then
  echo "NO"
  echo $count
fi

