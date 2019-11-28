#define ARRAY_SIZE 2

int* returnArray(){
  static int arr[ARRAY_SIZE] = {1,2};
  Serial.println(sizeof(arr));
  return arr;
}

void passArray(int arr[], int arrSize){
  for(int i=0; i<arrSize; i++){
    Serial.println(arr[i]);
  }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
//  int* arr = returnArray();
//  Serial.println(arr[0]);
//  Serial.println(arr[1]);
  int arr[ARRAY_SIZE] = {2,3};
  passArray(arr, ARRAY_SIZE);
}
