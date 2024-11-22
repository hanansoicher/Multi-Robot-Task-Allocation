
#include <../simpleble/include/simpleble_c/*.h>



int main(int argc, char** argv) {
   if (!SimpleBLE::Adapter::bluetooth_enabled()) {
      printf("Bluetooth is not enabled");
      return 1;
   }

   auto adapters = SimpleBLE::Adapter::get_adapters();
   if (adapters.empty()) {
      printf("No Bluetooth adapters found");
      return 1;
   }

   // Use the first adapter
   auto adapter = adapters[0];

   // Do something with the adapter
   std::cout << "Adapter identifier: " << adapter.identifier() << std::endl;
   std::cout << "Adapter address: " << adapter.address() << std::endl;

   return 0;
}

void scan_for_hm10(void) {

}