package artisynth.models.exReader;

import java.io.File;

import maspack.fileutil.DefaultConsoleFileTransferListener;
import maspack.fileutil.FileGrabber;
import maspack.fileutil.FileTransferListener;
import maspack.fileutil.uri.ExactMatcher;
import maspack.fileutil.uri.URIx;
import maspack.fileutil.vfs.EncryptedUserAuthenticator;
import maspack.fileutil.vfs.PasswordCryptor;

/**
 * Sets up remote source and adds authenticator
 * for protected data
 * @author antonio
 *
 */
public class CmissFileGrabber extends FileGrabber {

   // set to download from Antonio's CMISS directory
   private static final String CMISS_REMOTE_SOURCE = 
      "tgz:https://www.ece.ubc.ca/~antonios/files/phuman/cmiss_orig.tar.gz!/";
   
//   private URIx localZipUri = null;
   
   public CmissFileGrabber() {
      setRemoteSource(CMISS_REMOTE_SOURCE);
      setOptions(FileGrabber.DOWNLOAD_ZIP);  // download entire zip file first
      addAuthenticator();
      addListener();
   }
   
   public CmissFileGrabber(String downloadDir) {
      this();
      setDownloadDir(downloadDir);
   }
      
   private void addListener() {
      FileTransferListener listener = new DefaultConsoleFileTransferListener();
      addTransferListener(listener);
   }
   
   private void addAuthenticator() {
      
      ExactMatcher uriMatcher = 
         new ExactMatcher("https","www.ece.ubc.ca",
            "/~antonios/files/phuman/cmiss_orig.tar.gz");
      
      PasswordCryptor crypt = new PasswordCryptor();
      String hexKey = "E78B1B2D328CC997F531164CBB4A5EF5"; // default AES128 key
      try {
         crypt.setKey(hexKey);
      } catch (Exception e) {
         e.printStackTrace();
         return;
      } 
      
      EncryptedUserAuthenticator auth = new EncryptedUserAuthenticator(crypt);
      auth.setUserName("phuman");
      auth.setPassword("4F0263DF8BA45A8DBE19FD3717AF0ED4"); // encrypted password (phum@n@rtisynth)
      
      addUserAuthenticator(uriMatcher, auth);
      
   }
   
   
   
   
}
