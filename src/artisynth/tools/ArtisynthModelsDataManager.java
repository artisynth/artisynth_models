package artisynth.tools;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import javax.crypto.Cipher;

import artisynth.core.util.ArtisynthDataManager;
import artisynth.core.util.ArtisynthPath;
import maspack.crypt.Base64;
import maspack.crypt.GenericCryptor;
import maspack.json.JSONReader;

public class ArtisynthModelsDataManager extends ArtisynthDataManager {
   
   private static File publicConfig = ArtisynthPath.getSrcRelativeFile(ArtisynthModelsDataManager.class, ".owncloud.public.config");
   private static File privateConfig = ArtisynthPath.getSrcRelativeFile(ArtisynthModelsDataManager.class, ".private/.owncloud.private.config");
      
   public ArtisynthModelsDataManager() {
      super();
      loadConfig();
   }
   
   private void loadConfig() {
      if (privateConfig.exists()) {
         loadConfig(privateConfig);
      } else {
         loadConfig(publicConfig);
      }
   }
   
   private void loadConfig(File configFile) {
      JSONReader jreader = new JSONReader();
      Object json = null;
      try {
         json = jreader.read(configFile);
      } catch (FileNotFoundException e) {
         json = new HashMap<String,Object>();
      } finally {
         jreader.close();
      }
      
      // default to source-relative
      String remote_uri = (new File(ArtisynthPath.getHomeDir()+ "/src/").getAbsoluteFile()).toURI().toString();
      String local_dir = (new File(ArtisynthPath.getCacheDir(), "data/artisynth_models")).getAbsolutePath();
      String username = null;
      String password = null;
      String cipher = null;
      byte[] cipher_key = null;
      
      if (json instanceof Map<?,?>) {
         @SuppressWarnings("unchecked")
         Map<String,Object> jmap = (Map<String,Object>)json;
         for (Entry<String,Object> entry : jmap.entrySet()) {
            String key = entry.getKey();
            String val = entry.getValue().toString();
            if ("remote_uri".equals(key)) {
               remote_uri = val;
            } else if ("username".equals(key)) {
               username = val;
            } else if ("password".equals(key)) {
               password = val;
            } else if ("cipher".equals(key)) {
               cipher = val;
            } else if ("cipher_key".equals(key)) {
               cipher_key = Base64.decode(val);
            } else if ("local_dir".equals(key)) {
               local_dir = val;
            }
         }
      }
      
      super.setRemoteRoot(remote_uri);
      
      if (local_dir != null) {
         File f = ArtisynthPath.findFile(local_dir);
         if (f == null) {
            f = ArtisynthPath.getHomeRelativeFile(local_dir, ".");
         }
         if (f != null) {
            super.setLocalRoot(new File(local_dir));
         }
      }
      
      if (cipher != null) {
         try {
            Cipher ciph = Cipher.getInstance(cipher);
            GenericCryptor cryptor = new GenericCryptor(ciph);
            cryptor.setKey(cipher_key);
            setCryptor(cryptor);
         } catch (Exception e) {
            throw new RuntimeException(e);
         }
      }
      
      if (username != null && password != null) {
         setCredentials(username, password);
      }
      
   }
   
   public static void main(String[] args) {
      
      ArtisynthModelsDataManager manager = new ArtisynthModelsDataManager();
      File file = ArtisynthPath.getSrcRelativeFile(manager, "data/upload.txt");
      try {
         File test = manager.getPackageRelativeFile(manager, "test.txt");
         if (test != null && test.exists()) {
            System.out.println("Download succeeded");
            test.delete();
         } else {
            throw manager.getManager().getLastException();
         }
         manager.putPackageRelativeFile(file, manager, file.getName());
         System.out.println("Upload succeeded");
      } catch (Exception e) {
         e.printStackTrace();
      }
      
   }

}
