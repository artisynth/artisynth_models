package artisynth.tools.exReader;

import maspack.crypt.AESCryptor;
import maspack.fileutil.DefaultConsoleFileTransferListener;
import maspack.fileutil.FileManager;
import maspack.fileutil.FileTransferListener;
import maspack.fileutil.uri.ExactMatcher;
import maspack.fileutil.vfs.EncryptedUserAuthenticator;

/**
 * Sets up remote source and adds authenticator
 * for protected data
 * @author antonio
 *
 */
public class CmissFileManager extends FileManager {

   // set to download from Antonio's CMISS directory
   private static final String CMISS_REMOTE_SOURCE = 
      "tgz:https://www.ece.ubc.ca/~antonios/files/phuman/cmiss_orig.tar.gz!/";
   
//   private URIx localZipUri = null;
   
   public CmissFileManager() {
      setRemoteSource(CMISS_REMOTE_SOURCE);
      setOptions(FileManager.DOWNLOAD_ZIP);  // download entire zip file first
      addAuthenticator();
      addListener();
   }
   
   public CmissFileManager(String downloadDir) {
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
      
      AESCryptor crypt = new AESCryptor();
      String hexKey = "E78B1B2D328CC997F531164CBB4A5EF5"; // default AES128 key
      try {
         crypt.setHexKey(hexKey);
      } catch (Exception e) {
         e.printStackTrace();
         return;
      } 
      
      EncryptedUserAuthenticator auth = new EncryptedUserAuthenticator(crypt);
      auth.setUserName("phuman");
      auth.setEncryptedPassword("4F0263DF8BA45A8DBE19FD3717AF0ED4"); // encrypted password
      
      addUserAuthenticator(uriMatcher, auth);
      
   }
   
   
   
   
}
