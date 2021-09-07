cd ../../libs/ompl/
for commit in $(git rev-list main) 
do
    FILES=`git ls-files | grep -e "geometric/planners/.*\.h"`
    for f in $FILES
    do
      echo $f
    done
    echo `echo $FILES |wc -l`
    echo `git log --format=%B -n 1 $commit`
    echo `git show -s --format=%ci $commit`
    exit 0
    # echo $commit
done

