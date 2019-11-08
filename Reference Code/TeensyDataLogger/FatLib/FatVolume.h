/**
 * Copyright (c) 2011-2019 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#ifndef FatVolume_h
#define FatVolume_h
#include "FatPartition.h"
#include "FatFile.h"
/**
 * \file
 * \brief FatVolume class
 */
//------------------------------------------------------------------------------
/**
 * \class FatVolume
 * \brief Integration class for the FatLib library.
 */
class FatVolume : public  FatPartition {
 public:
  /**
   * Initialize an FatVolume object.
   * \param[in] dev Device block driver.
   * \param[in] setCwv Set current working volume if true.
   * \param[in] part partition to initialize.
   * \return true for success or false for failure.
   */
  bool begin(BlockDevice* dev, bool setCwv = true, uint8_t part = 1) {
    if (!init(dev, part)) {
      return false;
    }
    if (!chdir()) {
      return false;
    }
    if (setCwv) {
      m_cwv = this;
    }
    return true;
  }
  /** Change global current working volume to this volume. */
  void chvol() {m_cwv = this;}
  /** \return current working volume. */
  static FatVolume* cwv() {return m_cwv;}
  /**
   * Set volume working directory to root.
   * \return true for success or false for failure.
   */
  bool chdir() {
    m_vwd.close();
    return m_vwd.openRoot(this);
  }
  /**
   * Set volume working directory.
   * \param[in] path Path for volume working directory.
   * \return true for success or false for failure.
   */
  bool chdir(const char *path);

  //----------------------------------------------------------------------------
  /**
   * Test for the existence of a file.
   *
   * \param[in] path Path of the file to be tested for.
   *
   * \return true if the file exists else false.
   */
  bool exists(const char* path) {
    FatFile tmp;
    return tmp.open(this, path, O_RDONLY);
  }
  //----------------------------------------------------------------------------
  /** List the directory contents of the volume root directory.
   *
   * \param[in] pr Print stream for list.
   *
   * \param[in] flags The inclusive OR of
   *
   * LS_DATE - %Print file modification date
   *
   * LS_SIZE - %Print file size.
   *
   * LS_R - Recursive list of subdirectories.
   *
   * \return true for success or false for failure.
   */
  bool ls(print_t* pr, uint8_t flags = 0) {
    return m_vwd.ls(pr, flags);
  }
  //----------------------------------------------------------------------------
  /** List the contents of a directory.
   *
   * \param[in] pr Print stream for list.
   *
   * \param[in] path directory to list.
   *
   * \param[in] flags The inclusive OR of
   *
   * LS_DATE - %Print file modification date
   *
   * LS_SIZE - %Print file size.
   *
   * LS_R - Recursive list of subdirectories.
   *
   * \return true for success or false for failure.
   */
  bool ls(print_t* pr, const char* path, uint8_t flags) {
    FatFile dir;
    return dir.open(this, path, O_RDONLY) && dir.ls(pr, flags);
  }
  //----------------------------------------------------------------------------
  /** Make a subdirectory in the volume root directory.
   *
   * \param[in] path A path with a valid name for the subdirectory.
   *
   * \param[in] pFlag Create missing parent directories if true.
   *
   * \return true for success or false for failure.
   */
  bool mkdir(const char* path, bool pFlag = true) {
    FatFile sub;
    return sub.mkdir(vwd(), path, pFlag);
  }
  //----------------------------------------------------------------------------
  /** open a file
   *
   * \param[in] path location of file to be opened.
   * \param[in] oflag open flags.
   * \return a File32 object.
   */
  File32 open(const char *path, oflag_t oflag = O_RDONLY) {
    File32 tmpFile;
    tmpFile.open(this, path, oflag);
    return tmpFile;
  }
  //----------------------------------------------------------------------------
  /** Remove a file from the volume root directory.
   *
   * \param[in] path A path with a valid name for the file.
   *
   * \return true for success or false for failure.
   */
  bool remove(const char* path) {
    FatFile tmp;
    return tmp.open(this, path, O_WRONLY) && tmp.remove();
  }
  //----------------------------------------------------------------------------
  /** Rename a file or subdirectory.
   *
   * \param[in] oldPath Path name to the file or subdirectory to be renamed.
   *
   * \param[in] newPath New path name of the file or subdirectory.
   *
   * The \a newPath object must not exist before the rename call.
   *
   * The file to be renamed must not be open.  The directory entry may be
   * moved and file system corruption could occur if the file is accessed by
   * a file object that was opened before the rename() call.
   *
   * \return true for success or false for failure.
   */
  bool rename(const char *oldPath, const char *newPath) {
    FatFile file;
    return file.open(vwd(), oldPath, O_RDONLY) && file.rename(vwd(), newPath);
  }
  //----------------------------------------------------------------------------
  /** Remove a subdirectory from the volume's working directory.
   *
   * \param[in] path A path with a valid name for the subdirectory.
   *
   * The subdirectory file will be removed only if it is empty.
   *
   * \return true for success or false for failure.
   */
  bool rmdir(const char* path) {
    FatFile sub;
    return sub.open(this, path, O_RDONLY) && sub.rmdir();
  }
  //----------------------------------------------------------------------------
  /** Truncate a file to a specified length.  The current file position
   * will be at the new EOF.
   *
   * \param[in] path A path with a valid name for the file.
   * \param[in] length The desired length for the file.
   *
   * \return true for success or false for failure.
   */
  bool truncate(const char* path, uint32_t length) {
    FatFile file;
    return file.open(this, path, O_WRONLY) && file.truncate(length);
  }
#if ENABLE_ARDUINO_SERIAL
   /** List the directory contents of the root directory to Serial.
   *
   * \param[in] flags The inclusive OR of
   *
   * LS_DATE - %Print file modification date
   *
   * LS_SIZE - %Print file size.
   *
   * LS_R - Recursive list of subdirectories.
   *
   * \return true for success or false for failure.
   */
  bool ls(uint8_t flags = 0) {
    return ls(&Serial, flags);
  }
  /** List the directory contents of a directory to Serial.
   *
   * \param[in] path directory to list.
   *
   * \param[in] flags The inclusive OR of
   *
   * LS_DATE - %Print file modification date
   *
   * LS_SIZE - %Print file size.
   *
   * LS_R - Recursive list of subdirectories.
   *
   * \return true for success or false for failure.
   */
  bool ls(const char* path, uint8_t flags = 0) {
    return ls(&Serial, path, flags);
  }
#endif  // ENABLE_ARDUINO_SERIAL
#if ENABLE_ARDUINO_STRING
  //----------------------------------------------------------------------------
  /** open a file
   *
   * \param[in] path location of file to be opened.
   * \param[in] oflag open flags.
   * \return a File32 object.
   */
  File32 open(const String &path, oflag_t oflag = O_RDONLY) {
    return open(path.c_str(), oflag );
  }
#endif  // ENABLE_ARDUINO_STRING

 private:
  FatFile* vwd() {return &m_vwd;}
  friend FatFile;
  FatFile m_vwd;
  static FatVolume* m_cwv;
};
#endif  // FatVolume_h
