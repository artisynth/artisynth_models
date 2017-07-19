The Property Specification Language (PSL) can be hard to read in a regular text
file with a single font color. I've developed a quick-and-dirty way to use
Vim's syntax coloring features for PSL. Just follow these steps:

1. If needed, create the directories
      ~/.vim/syntax/
      ~/.vim/ftdetect/
   on Unix-based systems, or
      $HOME/vimfiles/syntax/
      $HOME/vimfiles/ftdetect/
   on Windows systems.
2. Copy the file "syntax/psl.vim" to the (new) "syntax" directory. 
3. Copy the file "ftdetect/psl.vim" to the (new) "ftdetect" directory.

If all went well, Vim will now highlight the syntax of PSL files, so long as these
files have the extension ".psl".