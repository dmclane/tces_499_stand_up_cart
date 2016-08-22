# MagicBox

Android Client for development

### How to setup the project on your machine! ###
* Find the box tht says [HTTPS][https://github.com/....]
* Copy the link. 
* Open git bash/shell and locate the directory that you would like the project to be on.
* Once you are in the directory, type: git clone _link-you-copied-earlier_
* Now that development is downloaded onto your machine, open AndroidStudio.
* In AndroidStudio, choose Open Existing Project. 
* Locate development and open it. 
* ...

### How to start a new feature ###
When starting a new feature/layout/change.

1. Go to your development branch. 
* git checkout Development
2. Make sure the project is "clean" with a git status. 
* git fetch
* git status
3. Do a git pull if your Development branch is behind in commits. 
* git pull
4. Create a new branch off the development branch. You can call it your own name and do work there. 
* git checkout -b newbranchname
* What this command does is it will create a new branch and also switch to it. 
6. Go to the new branch and make the necessary changes. (If you just did the above then you should already be on that branch and wont need to do this step.)
* git checkout newbranchname
7. Now that you've finish your changes, add and commit your changes. 
* git add .
* this command will add all your changes to the commit. 
* git commit -m "enter a important message. don't be lazy!" 
8. Git status to make sure that your branch is all good! 
* git status
9. Go back to the development(1/2) branch and see if its behind in the commits. If it is then do a pull and merge the development(1/2) branch into your new branch. This makes sure that your changes will work with the most updated development(1/2) branch that you branched off of earlier. 
* git checkout development(1/2)
* git fetch
* git status
* git pull
* git checkout newbranchname
* git merge development(1/2)
* make sure there are no conflicts. 
10. If there was no changes or if your newly updated branch has no issues, then you can finally merge your new branch into the development(1/2) branch. 
* git checkout development(1/2)
* git merge newbranchname
11. Lastly, push the development(1/2) branch up to the server.
* git status
* git push (if there are no issues!) 

### Stashing your work ###
Stashing your work will allow you to remove all your changes but also stash them somewhere incase you want to bring them back. To stash your work follow these instructions.
* stash your work with 'git stash -u'
* That's it! 
You can see what's on your stash stack by typing the following:
* git stash list
If you want to pop work off your stash to work on it again, type this:
* git stash pop
If you don't want what you stashed anymore, type this:
* git stash drop
