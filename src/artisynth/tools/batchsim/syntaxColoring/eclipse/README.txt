The Property Specification Language (PSL) can be hard to read in a regular text
file with a single font color. I've developed a quick-and-dirty way to use
Eclipse's syntax coloring features for PSL. Just follow these steps:

1. In Eclipse Marketplace, install Xtext (NOT Xtend) and any dependencies (including PDE).
2. Make a new Xtext project: File -> New -> Project... -> Xtext -> Xtext project.
3. Project name: org.propspeclang
4. Language name: org.propspeclang.PSL
5. Language extensions: psl
6. Click Finish. Several new projects are created.
7. Find and open the file "org.propspeclang/src/org.propspeclang/PSL.xtext" (the new
   project may in fact open the file automatically for you).
8. Copy the contents of the file "PSL.xtext" from this directory into that new file.
9. Right-click the new file and select Run As -> Generate Xtext Artifacts.
10. If all goes well, more files are automatically generated.
11. Right-click on the project org.propspeclang and select Run As -> Eclipse Application.
12. In the new instance of Eclipse that spawns, make a new Java project.
13. Call the project whatever you like, e.g. "PropSpecs".
14. Right-click the "src" directory, and select Import... -> General -> File System.
15. Click Next. In the next window, select the props file to import.
16. Expand "Advanced >>". Select "Create links in workspace" and "Create virtual folders"
17. Click Finish.

If all went well, there is now a link in this new instance of Eclipse to the original
file. Any changes here will be reflected in the original as well. If the file has the
extension ".psl", then Eclipse should be coloring the syntax within the file. It also
shows errors, but the error messages aren't that good.

Why do you need to start up a new Eclipse Application? Why can't you use it in the same
one as you normally use? Simple answer: because Xtext doesn't work that way. But once
you have followed the steps above, you only need to repeat Step 11 above to access your
*.psl files (and Steps 14-17 whenever you want to add a new file).

Note: if you use Vim, the Vim syntax highlighting is much more intuitive than in Eclipse.
