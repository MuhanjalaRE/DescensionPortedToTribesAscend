#pragma once
#include <vector>
#include <Windows.h>
namespace UE_Utilities {

	void getHWND(void);
	extern HWND hWnd;

	class Key {
		int key;
		bool flag = false;
		void (*onState)(void);
		void (*onHoldState)(void);

		void (*offUnHoldState)(void);
		void (*offState)(void);

	public:
		Key(int key, void (*onState)(void) = NULL, void (*offUnHoldState)(void) = NULL, void (*onHoldState)(void) = NULL, void (*offState)(void) = NULL);
		bool checkState(bool checkhwnd = true);
	};

	class KeyManager {
		std::vector<Key> keys;
		std::vector<Key> pressedKeys;

	public:
		void addKey(Key key);
		void addKey(int key, void (*onState)(void) = NULL, void (*offUnHoldState)(void) = NULL, void (*onHoldState)(void) = NULL, void (*offState)(void) = NULL);
		void checkKeyStates(bool checkhwnd = true);
		std::vector<Key> getPressedKeys(void);
	};  // keyManager;

	extern KeyManager keyManager;
}